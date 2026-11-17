#!/usr/bin/env python3
"""Export reviewable diffs for the retained files against the pinned archives."""
import difflib
import hashlib
import json
from pathlib import Path
import tarfile
ROOT=Path(__file__).resolve().parents[1]

def main():
    pins=json.loads((ROOT/'third_party/UPSTREAMS.json').read_text());out=ROOT/'docs/upstream_diff';out.mkdir(exist_ok=True)
    report={}
    for name,pin in pins.items():
        archive=Path.home()/f'.cache/gazebo_autonomy/{name}.tar.gz'
        if hashlib.sha256(archive.read_bytes()).hexdigest()!=pin['archive_sha256']:raise ValueError('Archive checksum mismatch')
        with tarfile.open(archive) as tar:
            originals={m.name.split('/',1)[1]:m for m in tar if m.isfile() and '/' in m.name}
            patches=[];changed=[];unchanged=[]
            for path in sorted((ROOT/'third_party'/name).rglob('*')):
                if not path.is_file() or '__pycache__' in path.parts:continue
                relative=path.relative_to(ROOT/'third_party'/name).as_posix();data=path.read_bytes()
                member=originals.get(relative);old=tar.extractfile(member).read() if member else b''
                if old==data:unchanged.append(relative);continue
                changed.append(dict(path=relative,upstream_sha256=hashlib.sha256(old).hexdigest() if member else None,
                    local_sha256=hashlib.sha256(data).hexdigest()))
                try:
                    patches.extend(difflib.unified_diff([line+'\n' for line in old.decode().splitlines()],[line+'\n' for line in data.decode().splitlines()],
                        fromfile='a/'+relative if member else '/dev/null',tofile='b/'+relative))
                except UnicodeDecodeError:patches.append('Binary difference: '+relative+'\n')
        (out/(name+'.patch')).write_text(''.join(patches))
        report[name]=dict(revision=pin['revision'],selected_paths=pin['selected_paths'],changed=changed,unchanged=unchanged,
            patch_purpose='Review diff with normalized line endings; SHA256 covers original raw bytes.',
            scope='Only retained planner/message files are compared; unvendored upstream simulation utilities are excluded.')
    (out/'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({name:dict(changed=len(v['changed']),unchanged=len(v['unchanged'])) for name,v in report.items()}))
if __name__=='__main__':main()
