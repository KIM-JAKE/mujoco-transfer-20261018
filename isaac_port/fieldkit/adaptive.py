"""Callable M80 skill: observe, adapt pickups, retime, screen and test in PhysX.

The hardware Cell implementation remains a site integration. This module never
opens a robot socket and never labels a simulation plan as hardware-ready.
"""
from pathlib import Path
import argparse
import hashlib
import json
import math
import shutil
import subprocess
import sys
import time
from .vision import detect

PORT = Path(__file__).resolve().parents[1]
REPO = PORT.parent


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def feedback_contract(prepared, frozen):
    """Bind explicitly requested controller options to the frozen profile."""
    requested = prepared.get('experimental_controller', {})
    if not isinstance(requested, dict):
        raise ValueError('experimental_controller must be an object')
    if not requested:
        return {}
    allowed = {'pin_feedback_mode', 'pin_alignment_step_cap_rad',
               'pin_entry_settled', 'pin_entry_raw_pose'}
    if set(requested)-allowed:
        raise ValueError('Unsupported experimental controller option')
    if 'fr3-concurrent-profile.json' not in frozen:
        raise ValueError('Controller contract requires a frozen profile')
    profile = json.loads(frozen['fr3-concurrent-profile.json'])
    if not isinstance(profile, dict):
        raise ValueError('Controller profile must be an object')
    for name, expected in requested.items():
        if name in {'pin_entry_settled', 'pin_entry_raw_pose'}:
            if type(expected) is not bool:
                raise ValueError('Controller boolean required: '+name)
        elif name == 'pin_feedback_mode':
            if type(expected) is not str or expected not in {'legacy', 'grasp_compensated'}:
                raise ValueError('Unknown pin feedback mode')
        elif type(expected) not in (float, int) or not math.isfinite(expected) or not 0 < expected <= .0012:
            raise ValueError('Invalid pin alignment step cap')
        if name not in profile or type(profile[name]) is not type(expected) or profile[name] != expected:
            raise ValueError('Controller contract differs from frozen profile: '+name)
    return requested.copy()


class M80Skill:
    def __init__(self, baseline, output, *, python=sys.executable):
        self.baseline = Path(baseline).resolve()
        self.output = Path(output).resolve()
        self.python = str(python)

    def _run(self, script, arguments, log, *, check=True):
        with Path(log).open('w') as stream:
            subprocess.run([self.python, str(PORT/script), *map(str, arguments)],
                           cwd=REPO, stdout=stream, stderr=subprocess.STDOUT, check=check)

    def prepare(self, snapshot, camera_config, *, replay=False, respect_cutoff=False,
                conservative_ring_handoff=False, timing_mode='guarded'):
        """No ground-truth argument: only camera data and calibrated priors.

        Returns an offline-screened plan. Preparing a path is separate from
        production cycle timing and from real-robot commissioning.
        """
        started = time.monotonic()
        if timing_mode not in ('guarded','fast_v1'):
            raise ValueError('timing_mode must be guarded or fast_v1')
        if self.output.exists() and any(self.output.iterdir()):
            raise FileExistsError('Use a new plan directory; measured plans are immutable')
        self.output.mkdir(parents=True, exist_ok=True)
        cutoff = ['--respect-cutoff'] if respect_cutoff else []
        observations = self.output/'observations.json'
        snapshot=Path(snapshot).resolve();camera_config=Path(camera_config).resolve()
        for source,destination in [(snapshot,self.output/'capture.npz'),
                                   (snapshot.with_suffix('.json'),self.output/'capture.json'),
                                   (camera_config,self.output/'camera-config.json')]:
            shutil.copy2(source,destination)
        camera=json.loads((self.output/'camera-config.json').read_text())
        tracks_pin=camera.get('pin_holder_tracks_pin',False)
        if type(tracks_pin) is not bool:
            raise ValueError('pin_holder_tracks_pin must be an explicit boolean')
        stock_args=['--pin-holder-tracks-pin'] if tracks_pin else []
        detect(self.output/'capture.npz', self.output/'camera-config.json', observations, replay=replay)
        self._run('retarget_skill.py', ['--base', self.baseline,
                  '--observations', observations, '--output', self.output, *cutoff, *stock_args],
                  self.output/'retarget.log')
        self._run('verify_retargeted_skill.py', ['--base', self.baseline,
                  '--candidate', self.output, '--output', self.output/'pickup-contract.json'],
                  self.output/'pickup-contract.log')
        self._run('prepare_concurrent_timing.py', [
            '--base', self.output/'planning-base',
            '--output', self.output/'fr3-concurrent-profile.npz',
            '--aggressive-contact-timing', '--coordinated',
            '--left-ring-clear', '1.96', '--right-ring-clear', '3.4',
            '--left-pin-time-reserve', '1.04', '--right-return-time-reserve', '1.1',
            '--pin-approach-min', '.3' if timing_mode=='fast_v1' else '.45',
            '--pin-fine-min', '.35', '--button-min', '.2',
            *(['--pin-prealignment'] if timing_mode=='guarded' else []),
            '--strict-clamp-alignment',
            *(['--conservative-ring-handoff'] if conservative_ring_handoff else []),
            *cutoff], self.output/'timing.log')
        expected_controller=None
        if timing_mode=='fast_v1':
            expected_controller=dict(pin_feedback_mode='legacy',pin_entry_settled=False,
                                     pin_entry_raw_pose=False)
            profile_path=self.output/'fr3-concurrent-profile.json'
            profile=json.loads(profile_path.read_text())
            profile.update(expected_controller)
            profile_path.write_text(json.dumps(profile,indent=2)+'\n')
        checks = [
            ('audit_pin_pickup.py', ['--recording', self.output,
             '--plan', self.output/'arm-posture-fr3.npz',
             '--initial-scene', self.output/'estimated-scene.json'], 'pickup', 'preclosure_triangle_clear'),
            ('audit_spatial_candidate.py', ['--candidate', self.output,
             '--initial-scene', self.output/'estimated-scene.json', *cutoff], 'spatial', 'sampled_spatial_pass'),
            ('audit_concurrent_nominal.py', ['--base', self.output/'planning-base',
             '--plan-root', self.output, *cutoff], 'simultaneous', 'sampled_cross_arm_pass')]
        for script, args, name, key in checks:
            result = self.output/(name+'-audit.json')
            self._run(script, [*args, '--output', result], self.output/(name+'.log'))
            report = json.loads(result.read_text())
            if not report.get(key):
                raise RuntimeError(f'{name} screen failed; see {result}')
        product_names=[]
        if conservative_ring_handoff:
            schedule=json.loads((self.output/'independent_schedule.json').read_text())
            release={side:next(r['end_s']for r in schedule['lanes'][side]
                         if r['stage']['label']==label)for side,label in
                         [('left','release_last_pin'),('right','release_two_parts')]}
            final=max(r['end_s']for r in schedule['lanes']['right'])
            products=[('before',[0.,2.4],[0.,2.1]),
                      ('right_ring',[2.4,2.4],[2.1,release['right']]),
                      ('after',[2.4,release['left']],[release['right'],release['left']]),
                      ('button',[release['left'],release['left']],[release['left'],final])]
            for name,left,right in products:
                result=self.output/('delay-product-'+name+'.json')
                self._run('audit_crossing_product.py',['--candidate',self.output,
                          '--left',*left,'--right',*right,'--samples','101',
                          '--output',result,*cutoff],self.output/('delay-product-'+name+'.log'))
                if not json.loads(result.read_text())['sampled_cross_reference_product_pass']:
                    raise RuntimeError('Sampled delay product failed: '+str(result))
                product_names.append(result.name)
        names = ['capture.npz','capture.json','camera-config.json',
                 'observations.json', 'estimated-scene.json', 'pickup-contract.json', 'trajectory.npz',
                 'config.json', 'independent_schedule.json', 'retiming.json',
                 'arm-posture-fr3.npz', 'arm-posture-fr3.json',
                 'fr3-concurrent-profile.npz', 'fr3-concurrent-profile.json',
                 'pickup-audit.json', 'spatial-audit.json', 'simultaneous-audit.json']
        names+=product_names
        result = dict(state='offline_screened_requires_PhysX', hardware_ready=False,
                      conservative_ring_handoff=bool(conservative_ring_handoff),
                      timing_mode=timing_mode,
                      pin_alignment_strategy='legacy_moving_alignment' if timing_mode=='fast_v1' else 'prealignment_then_entry',
                      simulation_drive_damping_nm_s_rad={'left':800.,'right':1800.},
                      live_motion_enabled=False, replay=bool(replay),
                      output=str(self.output), planning_seconds=time.monotonic()-started,
                      files={n:sha(self.output/n) for n in names},
                      requires=['Native PhysX grasp/contact and actual FR3 motion acceptance',
                                'Fresh calibrated observation before cached-plan execution',
                                'Field base/TCP/payload and gripper/PLC/bimanual stop integration'])
        if timing_mode=='fast_v1':
            result['experimental_controller']=expected_controller
            result['requires'].append('Fast V1 has a recorded shifted-scene-A actual-limit failure; every new condition needs Native acceptance, with guarded replanning on rejection.')
        (self.output/'skill-plan.json').write_text(json.dumps(result, indent=2)+'\n')
        return result

    def verify_plan(self):
        report = json.loads((self.output/'skill-plan.json').read_text())
        if report['state'] != 'offline_screened_requires_PhysX':
            raise RuntimeError('Not a screened skill plan')
        for name, expected in report['files'].items():
            relative=Path(name)
            if relative.is_absolute() or '..' in relative.parts or not (self.output/relative).resolve().is_relative_to(self.output):
                raise RuntimeError('Unsafe plan member: '+name)
            if sha(self.output/name) != expected:
                raise RuntimeError('Plan changed after screening: '+name)
        return report

    def run_physx(self, run_directory, *, initial_scene, command_prefix=None,
                  mounted_repo=None, respect_cutoff=False,
                  arm_damping=800.,right_arm_damping=1800.):
        """Execute a NEW native test with separately supplied true initial poses.

        command_prefix can be an Isaac Python executable or docker-exec argv.
        mounted_repo maps this repository to its path in that runtime. The
        caller reserves its own GPU session. No shell interpolation is used.
        """
        prepared=self.verify_plan()
        frozen={name:(self.output/name).read_bytes() for name in prepared['files']}
        if any(hashlib.sha256(data).hexdigest()!=prepared['files'][name] for name,data in frozen.items()):
            raise RuntimeError('Plan changed while freezing execution inputs')
        controller = feedback_contract(prepared, frozen)
        if any(not math.isfinite(x) or x<=0 for x in [arm_damping,right_arm_damping]):
            raise ValueError('Positive finite simulation drive damping required')
        run = Path(run_directory).resolve()
        if run.exists():
            raise FileExistsError('Use a fresh simulation output directory')
        scene = Path(initial_scene).resolve()
        mounted = Path(mounted_repo).resolve() if mounted_repo else REPO
        def mapped(path):
            return str(mounted/Path(path).resolve().relative_to(REPO))
        args = [mapped(PORT/'test_concurrent_task.py'), '--output', mapped(run),
                '--reference-dir', mapped(self.output), '--plan-root', mapped(self.output),
                '--initial-scene', mapped(scene), '--feedback', '--precision', '--posture',
                '--fast', '--clamps', '--fr3-limits', '--efficient-timing',
                '--physical-sensors', '--soft-clamps', '--target-10', '--cycle-deadline', '22']
        args+=['--arm-damping',str(arm_damping),'--right-arm-damping',str(right_arm_damping)]
        if respect_cutoff:
            args.append('--respect-cutoff')
        run.parent.mkdir(parents=True, exist_ok=True)
        try:
            with run.with_suffix('.log').open('w') as stream:
                subprocess.run([*(command_prefix or [self.python]), *args], cwd=REPO,
                               stdout=stream, stderr=subprocess.STDOUT, check=True)
        finally:
            if run.exists():
                observed=run/'observed-plan';observed.mkdir()
                for name,data in frozen.items():
                    target=observed/name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
                (observed/'skill-plan.json').write_text(json.dumps(prepared,indent=2)+'\n')
        # Credit a measured episode only when native's own frozen q/clock and
        # reference files match the screened plan, even if the source changed.
        for name in ['arm-posture-fr3.npz','fr3-concurrent-profile.npz',
                     'fr3-concurrent-profile.json','trajectory.npz','config.json',
                     'independent_schedule.json','retiming.json']:
            if name not in frozen:continue
            native=run/name if (run/name).exists() else run/'recording'/name
            if not native.exists() or sha(native)!=prepared['files'][name]:
                raise RuntimeError('Native input differs from screened plan: '+name)
        if controller:
            acknowledgement = run/'feedback-parameters.json'
            if not acknowledgement.exists():
                raise RuntimeError('Native did not acknowledge requested controller options')
            actual = json.loads(acknowledgement.read_text())
            for name, expected in controller.items():
                if name not in actual or type(actual[name]) is not type(expected) or actual[name] != expected:
                    raise RuntimeError('Native controller differs from screened plan: '+name)
        for script in ['evaluate_fr3_limits.py', 'evaluate_contact.py',
                       'verify_contact_recording.py', 'audit_hardware_readiness.py']:
            self._run(script, [run], run/(Path(script).stem+'.log'),
                      check=script not in ['evaluate_contact.py','evaluate_fr3_limits.py'])
        result = json.loads((run/'acceptance.json').read_text())
        return result


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--snapshot', type=Path, required=True)
    p.add_argument('--camera-config', type=Path, required=True)
    p.add_argument('--replay', action='store_true')
    p.add_argument('--conservative-ring-handoff', action='store_true')
    p.add_argument('--timing-mode', choices=['guarded','fast_v1'], default='guarded')
    p.add_argument('--respect-cutoff', action='store_true')
    a = p.parse_args(argv)
    print(json.dumps(M80Skill(a.baseline,a.output).prepare(a.snapshot,a.camera_config,
          replay=a.replay,respect_cutoff=a.respect_cutoff,
          conservative_ring_handoff=a.conservative_ring_handoff,
          timing_mode=a.timing_mode),indent=2))


if __name__ == '__main__':
    main()
