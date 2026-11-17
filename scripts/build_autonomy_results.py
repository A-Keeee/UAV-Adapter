#!/usr/bin/env python3
"""Build a traceable six-combination comparison for the project README.

Requires complete runtime acceptance unless --allow-incomplete explicitly marks
the output as a development snapshot. Never changes flight records or scores.
"""
import argparse
from datetime import datetime, timezone, timedelta
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import statistics
import subprocess
import sys
import tempfile

from autonomy_provenance import ROOT, runtime_source_hashes
from trial_index import latest_configurations
from verify_closed_recording import current_evidence


def digest(data):
    return hashlib.sha256(data).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--matrix', required=True, type=Path)
    parser.add_argument('--faults', required=True, type=Path)
    parser.add_argument('--interfaces', required=True, type=Path)
    parser.add_argument('--os1-replay', type=Path, default=ROOT/'log/os1_channel_replay_results/latest')
    parser.add_argument('--regressions', type=Path, default=ROOT/'log/autonomy_regression_20261112/latest')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--allow-incomplete', action='store_true')
    args = parser.parse_args()
    # Always run the actual current auditor. An old green JSON must not bless
    # changed binaries, configuration, scenes or a newer failed fault attempt.
    with tempfile.TemporaryDirectory(prefix='autonomy_results_audit_') as temporary:
        audit_file = Path(temporary)/'audit.json'
        command = [sys.executable, str(ROOT/'scripts/audit_autonomy_release.py'),
                   '--matrix', str(args.matrix), '--faults', str(args.faults),
                   '--interfaces', str(args.interfaces), '--os1-replay', str(args.os1_replay),
                   '--regressions', str(args.regressions),
                   '--output', str(audit_file)]
        process = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if process.returncode not in (0, 1) or not audit_file.is_file():
            raise RuntimeError('Current release audit did not produce evidence: '+process.stderr[-2000:])
        audit_bytes = audit_file.read_bytes()
    matrix_bytes = (args.matrix/'verified_results.json').read_bytes()
    records = json.loads(matrix_bytes)
    audit = json.loads(audit_bytes)
    latest = latest_configurations(records)
    expected = set(itertools.product(('explore', 'forest'), ('mid360', 'avia', 'os1_64'), range(3)))
    if not expected <= set(latest):
        parser.error('Comparison requires all 18 configurations to have recorded results')
    accepted = sum(bool(latest[key].get('acceptance_pass')) for key in expected)
    verified = set(audit['verified_configurations'])
    if (audit['accepted_normal_configurations'] != accepted or
            audit['current_version_verified_configurations'] != len(verified) or
            audit['runtime_sources_sha256'] != runtime_source_hashes() or
            audit['matrix_index_sha256'] != digest(matrix_bytes)):
        parser.error('Audit is stale or inconsistent; run the release audit first')
    if not args.allow_incomplete and not audit.get('complete'):
        parser.error('Runtime acceptance is incomplete; use --allow-incomplete for an explicitly provisional snapshot')
    if audit.get('complete') and (accepted != 18 or len(verified) != 18):
        parser.error('Complete audit does not cover all configurations')
    runs = []
    for key in sorted(expected):
        record = latest[key]
        folder = Path(record['trial_directory'])/'mission'
        evaluation = Path(record.get('evaluation_file', folder/'evaluation/evaluation.json'))
        score_bytes = evaluation.read_bytes()
        score = json.loads(score_bytes)
        if score != record['data']['evaluation']:
            parser.error('Score differs from indexed evidence: '+str(evaluation))
        if bool(score.get('acceptance_pass')) != bool(record.get('acceptance_pass')):
            parser.error('Score and indexed acceptance differ: '+str(evaluation))
        label = '/'.join(map(str, key))
        if label in verified and not record.get('acceptance_pass'):
            parser.error('Current-version verification points to a failed trial: '+label)
        item = dict(configuration=label, task=key[0], simulated_payload=key[1], condition=key[2],
                    initial_yaw_rad=record['initial_yaw'], attempt=record.get('attempt', 1),
                    accepted=bool(record['acceptance_pass']), current_version_verified=label in verified,
                    mission_directory=str(folder.resolve()), evaluation_file=str(evaluation.resolve()),
                    evaluation_sha256=digest(score_bytes), score=score,
                    closed_recording_verified=current_evidence(folder/'bag', True))
        runs.append(item)
    if audit.get('complete') and not all(run['closed_recording_verified'] for run in runs):
        parser.error('A recording changed after the release audit; rerun verification')
    groups = []
    metrics = ['elapsed_airborne_sim_s', 'path_length_m', 'stops_over_2s', 'stopped_sim_s',
               'minimum_enroute_clearance_m', 'tracking_rmse_m', 'coverage', 'contact_pair_count']
    for task, payload in itertools.product(('explore', 'forest'), ('mid360', 'avia', 'os1_64')):
        selected = [r for r in runs if r['task'] == task and r['simulated_payload'] == payload]
        ranges = {}
        for metric in metrics:
            values = [r['score'].get(metric) for r in selected if r['score'].get(metric) is not None]
            if not values:
                continue
            if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
                parser.error('Non-finite metric: '+metric)
            ranges[metric] = dict(samples=len(values), minimum=min(values), median=statistics.median(values), maximum=max(values))
        errors = [v for r in selected for v in r['score'].get('waypoint_minimum_truth_errors_m', []) if v is not None]
        groups.append(dict(task=task, simulated_payload=payload, runs=len(selected),
                           latest_accepted=sum(r['accepted'] for r in selected),
                           current_version_verified=sum(r['current_version_verified'] for r in selected),
                           verified_closed_recordings=sum(r['closed_recording_verified'] for r in selected),
                           metric_ranges=ranges, maximum_waypoint_error_m=max(errors) if errors else None))
    created = datetime.now(timezone(timedelta(hours=8))).isoformat(timespec='seconds')
    result = dict(schema_version=1, created_at=created, runtime_acceptance_complete=bool(audit.get('complete')),
                  documentation_review_separate=True, latest_accepted_configurations=accepted,
                  current_version_verified_configurations=len(verified), recorded_result_count=len(records),
                  conditions='Initial yaw 0, +pi/4, -pi/4 in fixed scenes; not three map seeds',
                  selection='Latest attempt per configuration, including new failures',
                  report_tool_sha256=digest(Path(__file__).read_bytes()),
                  source_matrix_sha256=digest(matrix_bytes), source_audit_sha256=digest(audit_bytes),
                  groups=groups, runs=runs)
    # Complete evidence and formatting before creating the new output directory.
    serialized = json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False)+'\n'
    def link(path):
        return '<'+os.path.relpath(path, args.output.resolve())+'>'
    status = '运行验收已通过' if audit.get('complete') else '开发快照，尚未完成交付版本复验'
    lines = ['# 两类任务与三种雷达的结果对比', '', status, '',
             f'快照时间：{created}。最新配置通过 {accepted}/18，当前交付版本核验 {len(verified)}/18，累计 {len(records)} 份试验结果。', '',
             '每种组合为同一场景的三组初始航向（0、+45°、−45°），不是三个地图种子。以下区间为各组最新一次完整结果的最小值至最大值，不能用历史通过覆盖最新失败。', '',
             ('当前数据来自统一交付版本下的三组固定条件；载荷和扫描模式同时变化，不单独归因某个硬件参数。' if audit.get('complete') else
              '最终版本复验完成前，数据包含不同开发版本，不用于单独归因雷达性能差异。')+
             '飞行耗时为仿真时间；接口测试、故障测试和感知回放不计入正常飞行矩阵。', '',
             '本目录保存完整矩阵及发布核验的只读快照和 SHA256；部分旧失败录包已按维护清单清理，试验结果数不代表保留了所有旧原始点云。', '']
    names = {'mid360': 'MID-360', 'avia': 'Livox Avia', 'os1_64': 'Ouster OS1-64'}
    for group in groups:
        title = ('仓库探索与巡飞' if group['task'] == 'explore' else '树林 A→B')+' / '+names[group['simulated_payload']]
        lines += ['## '+title, '',
                  f"最新配置通过 {group['latest_accepted']}/3；当前版本核验 {group['current_version_verified']}/3；正常录包完整性／回放验证 {group['verified_closed_recordings']}/3。", '']
        for key, caption, scale, unit in [
                ('elapsed_airborne_sim_s', '飞行耗时', 1, '仿真秒'), ('path_length_m', '路径长度', 1, 'm'),
                ('stops_over_2s', '超过 2 秒的停顿次数', 1, '次'), ('minimum_enroute_clearance_m', '每次飞行的最小障碍间距', 1, 'm'),
                ('contact_pair_count', '障碍碰撞接触对', 1, '对'),
                ('tracking_rmse_m', '真值位置跟踪 RMSE', 1, 'm'), ('coverage', '保存地图覆盖率', 100, '%')]:
            if key in group['metric_ranges']:
                values = group['metric_ranges'][key]
                lines.append(f"- {caption}：{values['minimum']*scale:.3f}–{values['maximum']*scale:.3f} {unit}。")
        if group['maximum_waypoint_error_m'] is not None:
            lines.append(f"- 三次飞行中最大航点到达误差：{group['maximum_waypoint_error_m']:.3f} m。")
        lines.append('')
    lines += ['## 每次飞行的原始证据', '', '各次评分 JSON 另包含重规划耗时、建图耗时、CPU／RAM、缓存容量、接触对及失败原因；资源统计不含 Gazebo/PX4 和独立评分器。', '']
    for run in runs:
        label = run['configuration']
        version = '当前版本已核验' if run['current_version_verified'] else '待统一版本复验'
        lines.append(f"- {label}，尝试 {run['attempt']}，{version}：[独立评分]({link(run['evaluation_file'])})；[运行清单]({link(Path(run['mission_directory'])/'run.json')})。")
    lines += ['', '详细数值、每次评分内容及其 SHA256 见 [results.json](results.json)；来源快照为 [matrix_snapshot.json](matrix_snapshot.json) 和 [audit_snapshot.json](audit_snapshot.json)。', '']
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'matrix_snapshot.json').write_bytes(matrix_bytes)
    (args.output/'audit_snapshot.json').write_bytes(audit_bytes)
    (args.output/'results.json').write_text(serialized)
    (args.output/'README.md').write_text('\n'.join(lines))
    print(json.dumps(dict(output=str(args.output.resolve()), latest_accepted=accepted,
                          current_version_verified=len(verified), groups=len(groups), runs=len(runs),
                          runtime_acceptance_complete=bool(audit.get('complete')))))


if __name__ == '__main__':
    main()
