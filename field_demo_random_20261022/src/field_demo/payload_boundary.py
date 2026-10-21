"""Measured stopped-boundary payload bookkeeping; no automatic cycle activation.

setLoad describes the held object, never the separately configured Hand/tool.
The caller must use current calibrated camera/FK and confirmed Hand/image states.
No simulated part masses, inferred density or missing-inertia defaults exist.
"""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
from functools import wraps
import select
import subprocess
import time
import uuid
import numpy as np

from .calibration import rigid, utc_stamp


def parameters(row):
    if row.get('units') != 'kg,m,kg*m^2' or row.get('inertia_about') != 'center_of_mass':
        raise ValueError('Explicit SI mass/COM/COM-inertia convention required')
    mass = row['mass_kg']
    if type(mass) not in (int, float) or not np.isfinite(mass) or not 0 <= mass <= 5:
        raise ValueError('Measured finite mass required; no default mass')
    center = np.asarray(row['center_m'], dtype=float)
    inertia = np.asarray(row['inertia_column_major_kg_m2'], dtype=float)
    if center.shape != (3,) or inertia.shape != (9,) or not np.isfinite(center).all() or not np.isfinite(inertia).all():
        raise ValueError('Measured COM and all nine inertia entries required')
    if np.max(abs(center)) > 1:
        raise ValueError('COM exceeds one metre; check coordinate frame and units')
    inertia = inertia.reshape(3, 3, order='F')
    if not np.allclose(inertia, inertia.T, atol=1e-10, rtol=0):
        raise ValueError('Symmetric COM inertia required')
    eigen = np.linalg.eigvalsh(inertia)
    if eigen[0] < -1e-12 or eigen[2] > eigen[0]+eigen[1]+1e-10:
        raise ValueError('Physical PSD inertia and principal-moment triangle inequality required')
    if mass == 0:
        if np.any(center != 0) or np.any(inertia != 0):
            raise ValueError('Explicit empty payload requires zero COM/inertia')
    elif eigen[0] <= 0:
        raise ValueError('Positive mass requires positive measured inertia')
    return float(mass), center, inertia


def measured(row, root):
    parameters(row)
    record = row['measurement']
    if record.get('kind')=='model_derived':
        from .model_records import validate
        return validate(record,root,subject=row)
    if (record.get('verified') is not True or not str(record.get('by', '')).strip() or
            utc_stamp(record['at_utc']) > datetime.now(timezone.utc)):
        raise ValueError('Actual measured load provenance required')
    evidence = (Path(root)/record['evidence_file']).resolve()
    if not evidence.is_file() or not evidence.read_bytes().strip():
        raise ValueError('Nonempty actual mass/COM/inertia evidence required')
    return hashlib.sha256(evidence.read_bytes()).hexdigest()


def serial(mass, center, inertia):
    return dict(units='kg,m,kg*m^2', inertia_about='center_of_mass', mass_kg=float(mass),
                center_m=np.asarray(center).tolist(),
                inertia_column_major_kg_m2=np.asarray(inertia).ravel(order='F').tolist())


def object_in_flange(part, F_T_EE, T_tcp_part):
    if part.get('frame') != 'part':
        raise ValueError('Measured part COM/inertia must be expressed in its named part frame')
    mass, center, inertia = parameters(part)
    F_T_part = rigid(F_T_EE) @ rigid(T_tcp_part)
    R = F_T_part[:3, :3]
    return serial(mass, R@center+F_T_part[:3, 3], R@inertia@R.T)


def combine(baseline, part):
    if baseline.get('frame') != 'flange':
        raise ValueError('Measured empty payload must use flange F axes')
    ma, ca, ia = parameters(baseline)
    mb, cb, ib = parameters(part)
    mass = ma+mb
    if mass == 0:
        return serial(0., np.zeros(3), np.zeros((3, 3)))
    center = (ma*ca+mb*cb)/mass
    inertia = np.zeros((3, 3))
    for m, c, I in ((ma, ca, ia), (mb, cb, ib)):
        delta = c-center
        inertia += I+m*((delta@delta)*np.eye(3)-np.outer(delta, delta))
    return serial(mass, center, inertia)


def request_text(tool, before, after, F_T_EE):
    if tool.get('frame') != 'flange' or parameters(tool)[0] <= 0:
        raise ValueError('Configured tool parameters are separate measured flange-frame data')
    if parameters(tool)[0]+max(parameters(before)[0], parameters(after)[0]) > 3:
        raise ValueError('FR3 three-kg tool-plus-payload ceiling exceeded')
    rows = ['1']
    for row in (tool, before, after):
        m, c, I = parameters(row)
        rows.append(' '.join(format(float(x), '.17g') for x in [m, *c, *I.ravel(order='F')]))
    rows.append(' '.join(format(float(x), '.17g') for x in rigid(F_T_EE).ravel(order='F')))
    return '\n'.join(rows)+'\n'


def execute_boundary(site, binary, side, tool, before, after, F_T_EE):
    """One explicit real configuration operation, called only by the field policy.

    The explicit live field policy calls this only at stopped motion boundaries.
    A failed/ambiguous write inhibits further motion; there is no rollback.
    """
    text = request_text(tool, before, after, F_T_EE)
    path = site.log_dir/('payload-request-'+uuid.uuid4().hex+'.txt')
    path.write_text(text); path.chmod(0o444)
    with site.control_lock():
        if site.process is not None:
            raise RuntimeError('Payload change is forbidden while an arm process is active')
        site.require_ready()
        p = subprocess.Popen([str(binary), '--execute-set-load', site.config['robots'][side]['ip'], str(path)],
                             stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1)
        began = time.monotonic(); lines = []
        try:
            while p.poll() is None:
                site.require_ready()
                if time.monotonic()-began > 10:
                    raise TimeoutError('Payload write/readback timed out; load ownership is unknown')
                if select.select([p.stdout], [], [], .02)[0]:
                    line = p.stdout.readline(); lines.append(line)
                    if line.strip() == 'PAYLOAD_READY_REQUIRED':
                        site.require_ready()
                        p.stdin.write('READY\n'); p.stdin.flush()
            lines.extend(p.stdout.readlines()); error = p.stderr.read()
            if p.returncode:
                raise RuntimeError('Payload boundary rejected or unknown: '+error.strip())
            results = [json.loads(line) for line in lines if line.startswith('{')]
            if not results or results[-1].get('payload_set_and_read_back') is not True or results[-1].get('tool_unchanged') is not True:
                raise RuntimeError('No confirmed payload readback; inhibit motion')
            site.require_ready()
            result = results[-1]
            measured_after = dict(after, mass_kg=result['mass_kg'], center_m=result['F_x_Cload_m'],
                                  inertia_column_major_kg_m2=result['I_load_column_major_kg_m2'])
            if not np.allclose([parameters(after)[0], *parameters(after)[1], *parameters(after)[2].ravel()],
                               [parameters(measured_after)[0], *parameters(measured_after)[1], *parameters(measured_after)[2].ravel()],
                               atol=1e-9, rtol=0):
                raise RuntimeError('Readback differs from exact intended payload')
            # This existing field gate compares expected scalar mass with the
            # SDK at the next stopped stage. Never write the sealed site file.
            site.config['robots'][side]['load_mass_kg'] = after['mass_kg']
            site.event('payload_boundary_confirmed', side=side, request=str(path), before=before, after=after,
                       tool_changed=False, hardware_tested=False)
            return result
        except BaseException:
            site.cancelled.set()  # Do not guess success or continue to a motion.
            if p.poll() is None:
                p.terminate()
                try:p.wait(timeout=2)
                except subprocess.TimeoutExpired:p.kill();p.wait()
            raise
        finally:
            for stream in (p.stdin, p.stdout, p.stderr):stream.close()


def stopped_boundary(method):
    @wraps(method)
    def call(self, *args, **kwargs):
        with self.site.control_lock():
            if not self.valid or self.site.process is not None:
                raise RuntimeError('Known stopped cell boundary required; inhibit motion')
            self.site.require_ready()
            return method(self, *args, **kwargs)
    return call


class PayloadBoundaries:
    """Explicit hook API; never infers ownership from a commanded grasp alone."""
    def __init__(self, site, profile, root, *, apply=execute_boundary, facts=None):
        if profile.get('schema_version') != 1:
            raise ValueError('Explicit measured payload profile required')
        self.site, self.profile, self.root, self.apply = site, json.loads(json.dumps(profile, allow_nan=False)), Path(root), apply
        self._profile_hash = hashlib.sha256(json.dumps(self.profile, sort_keys=True).encode()).hexdigest()
        self.facts=facts
        self.current = {}; self.owner = {}; self.valid = True
        self.evidence = {}
        for side in ('left', 'right'):
            for name in ('tool', 'empty_payload'):
                row = profile['robots'][side][name]
                if row.get('frame') != 'flange':raise ValueError('Tool and empty payload require flange-frame data')
                self.evidence[side+'_'+name] = measured(row, root)
            self.current[side] = profile['robots'][side]['empty_payload'].copy()
            self.owner[side] = None
        for part in ('ring1', 'ring2', 'pin'):
            row = profile['parts'][part]
            if row.get('frame') != 'part':raise ValueError('Part-frame measured mass properties required')
            self.evidence[part] = measured(row, root)

    def _set(self, side, target, F_T_EE, owner):
        if not self.valid:raise RuntimeError('Payload ownership unknown; inhibit all motion')
        try:
            if hashlib.sha256(json.dumps(self.profile, sort_keys=True).encode()).hexdigest() != self._profile_hash:
                raise RuntimeError('Measured payload profile changed')
            # Recheck the exact measured data on every transition.
            for robot in ('left', 'right'):
                for name in ('tool', 'empty_payload'):
                    if measured(self.profile['robots'][robot][name], self.root) != self.evidence[robot+'_'+name]:
                        raise RuntimeError('Measured payload evidence changed')
            for part in ('ring1', 'ring2', 'pin'):
                if measured(self.profile['parts'][part], self.root) != self.evidence[part]:
                    raise RuntimeError('Measured part evidence changed')
            binary=(self.root/self.profile['driver_binary']).resolve()
            self.apply(self.site, binary, side, self.profile['robots'][side]['tool'],
                       self.current[side], target, F_T_EE)
            self.current[side], self.owner[side] = target, owner
        except BaseException:
            self.valid = False; self.site.cancelled.set(); raise

    @stopped_boundary
    def after_grasp(self, side, part, F_T_EE, T_tcp_part):
        if self.owner[side] is not None or not self.site.grippers[side].status()['is_grasped'] or not (self.facts.grasped(part,side,self.site.grippers[side].status()) if self.facts is not None else self.site.plc.read(part+'_grasped')):
            raise RuntimeError('Fresh actual Hand/image grasp and empty logical owner required')
        part_load = object_in_flange(self.profile['parts'][part], F_T_EE, T_tcp_part)
        self._set(side, combine(self.profile['robots'][side]['empty_payload'], part_load), F_T_EE, part)

    @stopped_boundary
    def refresh_held(self, side, part, F_T_EE, T_tcp_part):
        if self.owner[side] != part or not self.site.grippers[side].status()['is_grasped'] or not (self.facts.grasped(part,side,self.site.grippers[side].status()) if self.facts is not None else self.site.plc.read(part+'_grasped')):
            raise RuntimeError('Current actual held owner required before refreshing COM/inertia')
        part_load = object_in_flange(self.profile['parts'][part], F_T_EE, T_tcp_part)
        self._set(side, combine(self.profile['robots'][side]['empty_payload'], part_load), F_T_EE, part)

    @stopped_boundary
    def after_transfer_release(self, donor, receiver, part, frames, receiver_T_tcp_part, donor_signal, receiver_signal):
        if (donor == receiver or self.owner[donor] != part or self.owner[receiver] is not None or
                donor_signal == receiver_signal or self.site.grippers[donor].status()['is_grasped'] or
                not self.site.grippers[receiver].status()['is_grasped'] or
                (not self.facts.transfer_confirmed(part,donor,receiver) if self.facts is not None else (self.site.plc.read(donor_signal) or not self.site.plc.read(receiver_signal)))):
            raise RuntimeError('Confirmed donor release and independent receiver grasp required')
        part_load = object_in_flange(self.profile['parts'][part], frames[receiver], receiver_T_tcp_part)
        # There is no atomic dual-robot setLoad. Both arms stay stopped until
        # both readbacks succeed. A partial update inhibits the entire cell.
        self._set(receiver, combine(self.profile['robots'][receiver]['empty_payload'], part_load), frames[receiver], part)
        self._set(donor, self.profile['robots'][donor]['empty_payload'], frames[donor], None)

    @stopped_boundary
    def after_retained_release(self, side, part, F_T_EE):
        if (self.owner[side] != part or self.site.grippers[side].status()['is_grasped'] or
                not all(self.facts.fixture_fact(part,suffix) if self.facts is not None else self.site.plc.read(part+'_'+suffix) for suffix in ('seated', 'clamped', 'retained'))):
            raise RuntimeError('Confirmed seat, clamp, retention and actual Hand release required')
        self._set(side, self.profile['robots'][side]['empty_payload'], F_T_EE, None)

    @stopped_boundary
    def after_bin_return_release(self,side,part,F_T_EE,empty_proof):
        state=self.site.grippers[side].status()
        if self.owner[side]!=part or state['is_grasped']or state.get('width_m',0)<.078 or empty_proof.get('verified')is not True or len(empty_proof.get('source_capture_ids',[]))<2:
            raise RuntimeError('Actual release plus fresh broad two-view empty-Hand proof required')
        self._set(side,self.profile['robots'][side]['empty_payload'],F_T_EE,None)
