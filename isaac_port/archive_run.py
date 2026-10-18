"""Archive/restore a measured run with SHA-256 and no unrelated workspace files."""
from pathlib import Path, PurePosixPath
import argparse, hashlib, json, zipfile

FILES = ['acceptance.json','fr3-limits.json','hardware-readiness.json','run-manifest.json',
         'status.json','dynamics-audit.json','feedback-parameters.json','reference-profile.json',
         'recording-audit.json','dof-names.json','body-paths.json','tape-bodies.json',
         'clamp-events.json','button-trace.json','contacts.json','trace.json',
         'physics-tape.npz','joint-motion.npz','arm-posture-fr3.npz','arm-posture-fr3.json',
         'fr3-concurrent-profile.npz','fr3-concurrent-profile.json','configuration.usda','scene.usda']


def sha(data):return hashlib.sha256(data).hexdigest()


def pack(run, archive):
    paths=[run/name for name in FILES if (run/name).is_file()]
    paths+=sorted((run/'implementation').glob('*.py'))
    paths+=sorted(p for p in (run/'recording').glob('*') if p.suffix in ['.json','.npz'])
    if not (run/'run-manifest.json').exists():raise RuntimeError('Missing measured-run manifest')
    archive.parent.mkdir(parents=True,exist_ok=True)
    records=[]
    with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for path in paths:
            data=path.read_bytes();name=(Path(run.name)/path.relative_to(run)).as_posix()
            z.writestr(name,data);records.append(dict(path=name,bytes=len(data),sha256=sha(data)))
    report=dict(format='m80-measured-run-v1',run_name=run.name,archive_bytes=archive.stat().st_size,
                archive_sha256=sha(archive.read_bytes()),files=records,hardware_ready=False)
    archive.with_suffix('.manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(run=run.name,files=len(records),archive_bytes=report['archive_bytes'])))


def restore(archive,destination):
    report=json.loads(archive.with_suffix('.manifest.json').read_text())
    data=archive.read_bytes()
    if len(data)!=report['archive_bytes'] or sha(data)!=report['archive_sha256']:
        raise RuntimeError('Run archive checksum mismatch')
    destination=destination.resolve()
    expected={row['path']:row for row in report['files']}
    if len(expected)!=len(report['files']):raise RuntimeError('Duplicate manifest members')
    for name in expected:
        p=PurePosixPath(name)
        if p.is_absolute() or '..' in p.parts or not p.parts or p.parts[0]!=report['run_name']:
            raise RuntimeError('Unsafe member path')
    with zipfile.ZipFile(archive) as z:
        if len(z.infolist())!=len(expected) or set(z.namelist())!=set(expected):
            raise RuntimeError('Run archive inventory mismatch')
        # Check every member and destination conflict before writing anything.
        for info in z.infolist():
            row=expected[info.filename];member=z.read(info)
            if len(member)!=row['bytes'] or sha(member)!=row['sha256']:
                raise RuntimeError('Member checksum mismatch: '+info.filename)
            target=destination/info.filename
            if not target.resolve().is_relative_to(destination):raise RuntimeError('Destination symlink escape')
            if target.exists() and sha(target.read_bytes())!=row['sha256']:
                raise RuntimeError('Preserve differing existing file first: '+str(target))
        for info in z.infolist():
            target=destination/info.filename
            if target.exists():continue
            target.parent.mkdir(parents=True,exist_ok=True)
            with target.open('xb') as stream:stream.write(z.read(info))
    print(json.dumps(dict(state='restored',run_name=report['run_name'],checksums_passed=True,files=len(expected))))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='command',required=True)
    a=sub.add_parser('pack');a.add_argument('run',type=Path);a.add_argument('--archive',type=Path,required=True)
    b=sub.add_parser('restore');b.add_argument('archive',type=Path);b.add_argument('--destination',type=Path,default=Path(__file__).resolve().parent/'runs')
    args=p.parse_args()
    pack(args.run,args.archive) if args.command=='pack' else restore(args.archive,args.destination)
