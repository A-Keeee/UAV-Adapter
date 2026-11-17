#!/usr/bin/env python3
"""Retain a closed bag through a selected simulation time, with byte verification."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import sqlite3
import struct
import subprocess
import time

import psutil
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
import yaml


def digest(connection, cutoff):
    checksum = hashlib.sha256()
    counts = {}
    for mid, topic, stamp, data in connection.execute(
            'SELECT id,topic_id,timestamp,data FROM messages WHERE timestamp<=? ORDER BY id', (cutoff,)):
        checksum.update(struct.pack('<qqqq', mid, topic, stamp, len(data)))
        checksum.update(data)
        counts[topic] = counts.get(topic, 0)+1
    return checksum.hexdigest(), counts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--stop-sim-time', required=True, type=float)
    args = parser.parse_args()
    bag = args.bag.resolve()
    metadata = yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    files = metadata['relative_file_paths']
    if len(files) != 1 or metadata.get('compression_format'):
        raise RuntimeError('Expected one closed, uncompressed SQLite bag')
    source = bag/files[0]
    for process in psutil.process_iter():
        try:
            if any(Path(f.path) == source for f in process.open_files()):
                raise RuntimeError('Recording is still open in process '+str(process.pid))
        except (psutil.AccessDenied, psutil.NoSuchProcess):
            pass
    old = sqlite3.connect('file:'+str(source)+'?mode=ro', uri=True)
    topic = old.execute("SELECT id FROM topics WHERE name='/clock'").fetchone()[0]
    cutoff = None
    for stamp, data in old.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp', (topic,)):
        clock = deserialize_message(data, Clock).clock
        if clock.sec+clock.nanosec*1e-9 > args.stop_sim_time:
            cutoff = stamp
            break
    if cutoff is None:
        raise RuntimeError('Requested cutoff does not remove any recorded tail')
    temporary = bag.with_name(bag.name+'.trimmed')
    temporary.mkdir(exist_ok=False)
    try:
        new = sqlite3.connect(temporary/files[0])
        new.execute('ATTACH DATABASE ? AS original', (str(source),))
        schema = old.execute("SELECT type,name,sql FROM sqlite_master WHERE name NOT LIKE 'sqlite_%' ORDER BY type DESC").fetchall()
        for kind, name, sql in schema:
            if kind == 'table':
                new.execute(sql)
                if name == 'messages':
                    new.execute('INSERT INTO messages SELECT * FROM original.messages WHERE timestamp<=?', (cutoff,))
                elif name != 'metadata':
                    new.execute('INSERT INTO '+name+' SELECT * FROM original.'+name)
        for kind, name, sql in schema:
            if kind == 'index':
                new.execute(sql)
        new.commit()
        original_hash, original_counts = digest(old, cutoff)
        kept_hash, kept_counts = digest(new, cutoff)
        if original_hash != kept_hash or original_counts != kept_counts:
            raise RuntimeError('Retained serialized messages differ')
        if new.execute('PRAGMA integrity_check').fetchone()[0] != 'ok':
            raise RuntimeError('Trimmed SQLite integrity check failed')
        new.close()
        subprocess.run(['ros2', 'bag', 'reindex', str(temporary)], check=True)
        checked = yaml.safe_load((temporary/'metadata.yaml').read_text())['rosbag2_bagfile_information']
        if checked['message_count'] != sum(kept_counts.values()):
            raise RuntimeError('Reindexed metadata count differs')
        record = dict(reason='Remove redundant post-failure recording tail; retain all messages through cutoff',
            stop_sim_time=args.stop_sim_time, storage_timestamp_cutoff=cutoff,
            original_bytes=source.stat().st_size, retained_bytes=(temporary/files[0]).stat().st_size,
            original_messages=old.execute('SELECT COUNT(*) FROM messages').fetchone()[0],
            retained_messages=sum(kept_counts.values()), retained_serialized_messages_sha256=kept_hash,
            byte_identical_retained_messages=True, original_metadata=metadata)
        old.close()
        (temporary/'retention.json').write_text(json.dumps(record, indent=2)+'\n')
        previous = bag.with_name(bag.name+'.untrimmed')
        os.replace(bag, previous)
        os.replace(temporary, bag)
        shutil.rmtree(previous)
        print(json.dumps({k:v for k,v in record.items() if k!='original_metadata'}), flush=True)
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)


if __name__ == '__main__':
    main()
