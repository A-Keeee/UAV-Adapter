#!/usr/bin/env python3
"""Lossless ROS 2 file compression; verify bytes before removing plain SQLite."""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import subprocess

import psutil
import yaml


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4*1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bags', nargs='+', type=Path)
    args = parser.parse_args()
    for bag in args.bags:
        bag = bag.resolve()
        metadata_path = bag/'metadata.yaml'
        document = yaml.safe_load(metadata_path.read_text())
        info = document['rosbag2_bagfile_information']
        if info.get('compression_format'):
            continue
        # Killing a WAL-mode writer can leave committed messages outside the
        # main database. Do not archive/delete that database until SQLite has
        # finalized it; a byte-identical copy of the main file is insufficient.
        if list(bag.glob('*-wal')) or list(bag.glob('*-shm')):
            raise RuntimeError('Recording has unfinalized SQLite sidecars; originals retained')
        if any(Path(path).name != path for path in info['relative_file_paths']):
            raise RuntimeError('Recording fragment paths must stay inside the bag directory')
        originals = [bag/path for path in info['relative_file_paths']]
        for process in psutil.process_iter():
            try:
                if any(Path(f.path) in originals for f in process.open_files()):
                    raise RuntimeError('Recording still open in process '+str(process.pid))
            except (psutil.AccessDenied, psutil.NoSuchProcess):
                pass
        records = []
        for source in originals:
            target = source.with_name(source.name+'.zstd')
            temporary = target.with_name(target.name+'.partial')
            # A bounded finalizer may have stopped after a previous fragment.
            # Reuse an existing target only after checking its exact decoded
            # bytes against the still-retained original.
            candidate = target if target.exists() else temporary
            if not target.exists():
                subprocess.run(['zstd', '-T2', '-3', '--quiet', '--force', str(source), '-o', str(temporary)], check=True)
            digest = hashlib.sha256()
            decompressor = subprocess.Popen(['zstd', '--decompress', '--stdout', '--quiet', str(candidate)], stdout=subprocess.PIPE)
            for block in iter(lambda: decompressor.stdout.read(4*1024*1024), b''):
                digest.update(block)
            decompressor.stdout.close()
            if decompressor.wait() or digest.hexdigest() != sha256(source):
                raise RuntimeError('Lossless verification failed: '+str(source))
            if candidate == temporary:
                os.replace(temporary, target)
            records.append(dict(original_file=source.name, compressed_file=target.name,
                original_bytes=source.stat().st_size, compressed_bytes=target.stat().st_size,
                original_sha256=digest.hexdigest(), decompressed_bytes_verified=True))
        new = copy.deepcopy(document)
        new_info = new['rosbag2_bagfile_information']
        new_info['compression_format'] = 'zstd'
        new_info['compression_mode'] = 'FILE'
        new_info['relative_file_paths'] = [r['compressed_file'] for r in records]
        for file in new_info.get('files', []):
            file['path'] += '.zstd'
        (bag/'compression.json').write_text(json.dumps(dict(original_metadata=document, files=records), indent=2)+'\n')
        temporary = metadata_path.with_suffix('.yaml.partial')
        temporary.write_text(yaml.safe_dump(new, sort_keys=False))
        os.replace(temporary, metadata_path)
        for source in originals:
            source.unlink()
        print(json.dumps(dict(bag=str(bag), original_bytes=sum(r['original_bytes'] for r in records),
            compressed_bytes=sum(r['compressed_bytes'] for r in records), verified=True)), flush=True)


if __name__ == '__main__':
    main()
