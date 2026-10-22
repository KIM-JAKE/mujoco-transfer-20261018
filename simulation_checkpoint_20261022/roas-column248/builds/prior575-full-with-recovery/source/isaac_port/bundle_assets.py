"""Pack/restore a hash-checked PhysX seed without the full LFS migration.

Only explicit model assets, generated contact inputs and converted USD are
included. Runtime caches, Python environments and credentials are excluded.
"""
from pathlib import Path, PurePosixPath
import argparse
import hashlib
import json
import shutil
import tempfile
import zipfile

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
ASSETS = HERE/'assets'
MANIFEST = ASSETS/'physx-seed.json'


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest() if hasattr(hashlib, 'file_digest') else hashlib.sha256(stream.read()).hexdigest()


def pack():
    source = json.loads((HERE/'generated/cap_button/source.json').read_text())
    paths = {REPO.parent/item['path'] for item in source['assets']}
    for directory in ['generated/cap_button', 'generated/contact_task']:
        paths.update(p for p in (HERE/directory).rglob('*') if p.is_file() and
                     p.suffix in {'.json','.npz','.xml','.usd','.usda','.usdc','.png','.jpg','.obj','.brep'})
    paths.update([ASSETS/'NotoSansKR.ttf', ASSETS/'OFL.txt'])
    records = []
    for path in sorted(paths):
        relative = path.resolve().relative_to(REPO.resolve()).as_posix()
        records.append(dict(path=relative, bytes=path.stat().st_size, sha256=digest(path)))
    ASSETS.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='m80-seed-') as temp:
        archive = Path(temp)/'seed.zip'
        with zipfile.ZipFile(archive, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as z:
            for item in records:
                info = zipfile.ZipInfo(item['path'], date_time=(2026,10,18,0,0,0))
                info.compress_type = zipfile.ZIP_DEFLATED
                info.external_attr = 0o100644 << 16
                z.writestr(info, (REPO/item['path']).read_bytes())
        chunks = []
        with archive.open('rb') as stream:
            index = 0
            while data := stream.read(64*1024*1024):
                path = ASSETS/f'physx-seed.zip.part-{index:02d}'
                path.write_bytes(data)
                chunks.append(dict(path=path.name, bytes=len(data), sha256=digest(path)))
                index += 1
    manifest = dict(format='m80-physx-seed-v1', files=records, chunks=chunks,
                    scope='Prepared M80 contact scene, original referenced meshes, USD, contact inputs and licensed render font; no environments or SAM weights',
                    hardware_ready=False)
    MANIFEST.write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(dict(files=len(records),compressed_bytes=sum(c['bytes'] for c in chunks),chunks=len(chunks))))


def restore(destination):
    manifest = json.loads(MANIFEST.read_text())
    destination = destination.resolve()
    expected = {item['path']:item for item in manifest['files']}
    for name in expected:
        path = PurePosixPath(name)
        if path.is_absolute() or '..' in path.parts:
            raise RuntimeError('Unsafe archive path')
    with tempfile.TemporaryDirectory(prefix='m80-restore-') as temp:
        archive = Path(temp)/'seed.zip'
        with archive.open('wb') as stream:
            for chunk in manifest['chunks']:
                path = ASSETS/chunk['path']
                if path.stat().st_size != chunk['bytes'] or digest(path) != chunk['sha256']:
                    raise RuntimeError('Seed archive checksum mismatch: '+path.name)
                with path.open('rb') as source:
                    shutil.copyfileobj(source,stream)
        with zipfile.ZipFile(archive) as z:
            if len(z.infolist()) != len(expected) or set(z.namelist()) != set(expected):
                raise RuntimeError('Unexpected archive member inventory')
            # Validate all bytes and destination conflicts before writing any files.
            for info in z.infolist():
                item=expected[info.filename]
                if info.file_size != item['bytes']:
                    raise RuntimeError('Member size mismatch: '+info.filename)
                data=z.read(info)
                if hashlib.sha256(data).hexdigest()!=item['sha256']:
                    raise RuntimeError('Member checksum mismatch: '+info.filename)
                target=destination/info.filename
                if not target.resolve().is_relative_to(destination):
                    raise RuntimeError('Destination symlink escapes clone')
                if target.exists() and digest(target)!=item['sha256']:
                    raise RuntimeError('Existing file differs; preserve it before restore: '+str(target))
            for info in z.infolist():
                target=destination/info.filename
                if target.exists():continue
                target.parent.mkdir(parents=True,exist_ok=True)
                with z.open(info) as source, target.open('xb') as stream:
                    shutil.copyfileobj(source,stream)
    print(json.dumps(dict(state='restored',files=len(expected),destination=str(destination),checksums_passed=True)))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('command',choices=['pack','restore'])
    p.add_argument('--destination',type=Path,default=REPO)
    a=p.parse_args()
    pack() if a.command=='pack' else restore(a.destination)
