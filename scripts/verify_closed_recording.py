#!/usr/bin/env python3
"""Verify closed ROS 2 recordings without modifying their source fragments.

Decompress one fragment at a time in a temporary directory. Check SQLite
integrity, exact topic counts, and ROS deserialization at both ends of every
topic in every fragment. This is storage/replay verification, not a flight
acceptance score or an all-message semantic check.
"""
import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import tempfile
import time

import psutil
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import yaml

from trial_index import latest_configurations, latest_fault_configurations

REPORT = 'replay_verification.json'
BASE_TOPICS = {'/autonomy/raw_cloud', '/autonomy/odom', '/clock', '/tf',
               '/tf_static', '/autonomy/status', '/ganyu/status'}
FLIGHT_TOPICS = {'/autonomy/native_trajectory', '/ganyu/control_reference'}


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def snapshot(path):
    stat = path.stat()
    return dict(bytes=stat.st_size, mtime_ns=stat.st_mtime_ns,
                sha256=sha256(path))


def atomic_json(path, value):
    temporary = path.with_name(path.name + '.partial')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    os.replace(temporary, path)


def ensure_closed(bag):
    for process in psutil.process_iter():
        try:
            if any(Path(item.path).parent == bag for item in process.open_files()):
                raise RuntimeError('Recording is open in process ' + str(process.pid))
        except (psutil.AccessDenied, psutil.NoSuchProcess):
            pass
    if list(bag.glob('*-wal')) or list(bag.glob('*-shm')):
        raise RuntimeError('Recording has unfinalized SQLite sidecars')


def current_evidence(bag, require_flight=False):
    """Lightweight gate after verification; changes invalidate the proof."""
    try:
        if list(bag.glob('*-wal')) or list(bag.glob('*-shm')):
            return False
        report = json.loads((bag / REPORT).read_text())
        if not report.get('passed') or report.get('verifier_sha256') != sha256(Path(__file__)):
            return False
        if require_flight and not report.get('full_flight_topics_verified'):
            return False
        if snapshot(bag / 'metadata.yaml') != report['metadata']:
            return False
        for name, proof in report['source_fragments'].items():
            stat = (bag / name).stat()
            if stat.st_size != proof['bytes'] or stat.st_mtime_ns != proof['mtime_ns']:
                return False
        for typename, schema in report['message_schemas'].items():
            if dict(get_message(typename).get_fields_and_field_types()) != schema:
                return False
        return True
    except (OSError, ValueError, KeyError, ImportError, AttributeError):
        return False


def verify(bag, require_flight=False, reuse=False, timeout=900):
    bag = bag.resolve()
    started = time.monotonic()
    deadline = started + timeout
    ensure_closed(bag)
    metadata_path = bag / 'metadata.yaml'
    metadata_proof = snapshot(metadata_path)
    info = yaml.safe_load(metadata_path.read_text())['rosbag2_bagfile_information']
    if info['storage_identifier'] != 'sqlite3':
        raise ValueError('Only SQLite recordings are supported by this verifier')
    compressed = bool(info.get('compression_format'))
    if compressed and (info['compression_format'] != 'zstd' or info['compression_mode'].upper() != 'FILE'):
        raise ValueError('Expected uncompressed or FILE zstd recording')
    names = info['relative_file_paths']
    if not names or len(set(names)) != len(names):
        raise ValueError('Missing or duplicate fragments')
    for name in names:
        if Path(name).name != name or not (bag / name).is_file():
            raise ValueError('Invalid fragment path: ' + name)
    sources = {name: snapshot(bag / name) for name in names}
    if reuse and current_evidence(bag, require_flight):
        old = json.loads((bag / REPORT).read_text())
        if old['source_fragments'] == sources:
            return old
    expected = {}
    types = {}
    for entry in info['topics_with_message_count']:
        topic = entry['topic_metadata']
        name = topic['name']
        if name in expected or topic['serialization_format'] != 'cdr':
            raise ValueError('Duplicate topic or unsupported serialization: ' + name)
        expected[name] = entry['message_count']
        types[name] = topic['type']
    required = BASE_TOPICS | (FLIGHT_TOPICS if require_flight else set())
    if any(expected.get(name, 0) <= 0 for name in required):
        raise ValueError('Required replay topics are missing or empty: ' + ','.join(sorted(required)))
    classes = {name: get_message(typename) for name, typename in types.items()}
    schemas = {types[name]: dict(cls.get_fields_and_field_types()) for name, cls in classes.items()}
    counts = Counter()
    fragments = []
    for name in names:
        if time.monotonic() >= deadline:
            raise TimeoutError('Recording verification deadline exceeded')
        source = bag / name
        with tempfile.TemporaryDirectory(prefix='autonomy_replay_') as temporary:
            database = source
            if compressed:
                database = Path(temporary) / 'fragment.db3'
                subprocess.run(['zstd', '--decompress', '--quiet', str(source), '-o', str(database)],
                               check=True, timeout=max(1, deadline-time.monotonic()))
            connection = sqlite3.connect(database.as_uri() + '?mode=ro&immutable=1', uri=True)
            try:
                connection.set_progress_handler(lambda: int(time.monotonic() >= deadline), 100000)
                if connection.execute('PRAGMA integrity_check').fetchall() != [('ok',)]:
                    raise ValueError('SQLite integrity check failed: ' + name)
                topics = dict()
                for topic_id, topic_name, typename, serialization in connection.execute(
                        'SELECT id,name,type,serialization_format FROM topics'):
                    if types.get(topic_name) != typename or serialization != 'cdr':
                        raise ValueError('Topic schema differs from metadata: ' + topic_name)
                    topics[topic_id] = topic_name
                local_counts = Counter()
                for topic_id, count in connection.execute('SELECT topic_id,count(*) FROM messages GROUP BY topic_id'):
                    if topic_id not in topics:
                        raise ValueError('Messages reference an undeclared topic')
                    topic_name = topics[topic_id]
                    local_counts[topic_name] += count
                    for order in ('ASC', 'DESC'):
                        row = connection.execute(
                            'SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp ' + order + ',id ' + order + ' LIMIT 1',
                            (topic_id,)).fetchone()
                        deserialize_message(row[0], classes[topic_name])
                counts.update(local_counts)
                fragments.append(dict(path=name, sqlite_integrity='ok', message_count=sum(local_counts.values()),
                                      per_topic_counts=dict(local_counts), endpoint_deserialization_passed=True))
            finally:
                connection.close()
    if dict(counts) != {name: count for name, count in expected.items() if count}:
        raise ValueError('Recorded topic counts differ from metadata')
    if sum(counts.values()) != info['message_count']:
        raise ValueError('Recorded total count differs from metadata')
    ensure_closed(bag)
    if metadata_proof != snapshot(metadata_path) or sources != {name: snapshot(bag / name) for name in names}:
        raise RuntimeError('Source recording changed during verification')
    result = dict(passed=True, bag=str(bag), verifier_sha256=sha256(Path(__file__)),
                  metadata=metadata_proof, source_fragments=sources, message_schemas=schemas,
                  total_messages=sum(counts.values()), per_topic_counts=dict(counts), fragments=fragments,
                  full_flight_topics_verified=all(counts[name] > 0 for name in FLIGHT_TOPICS),
                  elapsed_wall_seconds=time.monotonic()-started,
                  scope='SQLite integrity, exact counts, zstd decoding, first/last ROS message per topic per fragment')
    atomic_json(bag / REPORT, result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', nargs='?', type=Path)
    parser.add_argument('--matrix', type=Path)
    parser.add_argument('--faults', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--require-flight', action='store_true')
    parser.add_argument('--reuse-verified', action='store_true')
    parser.add_argument('--keep-going', action='store_true', help='Report all selected failures; exit nonzero if any fail')
    parser.add_argument('--wall-timeout', type=float, default=900, help='Per-bag verification deadline in seconds')
    args = parser.parse_args()
    selected = []
    if args.bag:
        selected.append((str(args.bag), args.bag, args.require_flight))
    if args.matrix:
        records = latest_configurations(json.loads((args.matrix / 'verified_results.json').read_text()))
        for key, record in sorted(records.items()):
            if not record.get('acceptance_pass'):
                raise ValueError('Latest normal trial has not passed: ' + str(key))
            selected.append(('/'.join(map(str, key)), Path(record['trial_directory']) / 'mission/bag', True))
    if args.faults:
        records = latest_fault_configurations([json.loads(p.read_text()) for p in args.faults.glob('*/fault_record.json')])
        for key, record in sorted(records.items()):
            if not record.get('fault_check_pass'):
                raise ValueError('Latest fault trial has not passed: ' + str(key))
            selected.append(('/'.join(key), Path(record['trial_directory']) / 'mission/bag', False))
    if not selected:
        parser.error('Provide a closed bag or validation matrix/fault directory')
    results = []
    for label, bag, full in selected:
        try:
            result = verify(bag, full, args.reuse_verified, args.wall_timeout)
            item = dict(label=label, bag=str(bag.resolve()), passed=result['passed'], messages=result['total_messages'])
        except Exception as error:
            if not args.keep_going:
                raise
            item = dict(label=label, bag=str(bag.resolve()), passed=False, error=str(error))
        results.append(item)
        print(json.dumps(item), flush=True)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            atomic_json(args.output, dict(complete=len(results) == len(selected) and all(r['passed'] for r in results), recordings=results))
    if any(not result['passed'] for result in results):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
