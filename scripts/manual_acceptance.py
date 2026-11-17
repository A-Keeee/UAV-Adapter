#!/usr/bin/env python3
"""User-operated acceptance entry: plan, run/resume, status and graceful stop."""
import argparse
from datetime import datetime
import fcntl
import hashlib
import itertools
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import psutil
from autonomy_provenance import ROOT, fingerprint
import acceptance_checkpoints as checkpoints
from trial_index import latest_configurations, latest_fault_configurations

BASE = ROOT/'log/manual_acceptance'
OLD_MATRIX = ROOT/'log/matrix/20261111_153325'
OLD_FAULTS = ROOT/'log/fault_trials/20261112_release'
OLD_INTERFACES = ROOT/'log/interface_checks_20261112_release'
HANDOFF = ROOT/'log/manual_acceptance_handoff/handoff.json'
PYTHON = '/usr/bin/python3'


def signature():
    assets = list((ROOT/'assets/gazebo_world').glob('*.sdf'))
    assets += list((ROOT/'assets/gazebo_models').glob('x500_*/model.sdf'))
    return dict(software={backend: fingerprint(backend) for backend in ('swarm', 'v2')},
        scenarios={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(assets)})


def state(folder):
    path = folder/'state.json'
    return json.loads(path.read_text()) if path.exists() else {}


def nonflight_limit(jobs):
    physical_entries={'run_autonomy_matrix.py','run_autonomy_faults.sh'}
    return next((index for index,job in enumerate(jobs) if
        job.get('requires_flight') or any(Path(arg).name in physical_entries for arg in job['command'])),len(jobs))


def historical_configurations(source):
    """Preserve task progress separately from current-version certification."""
    completed = set(source.get('historical_completed_configurations', []))
    completed.update(source.get('current_version_flights_reused', []))
    if source.get('interfaces'):
        origin = Path(source.get('checkpoint_directory', str(Path(source['interfaces']).parent)))
        if (origin/'jobs.json').exists():
            jobs = json.loads((origin/'jobs.json').read_text())
            completed.update(j['name'].removeprefix('完整飞行 ') for j in jobs[:state(origin).get('completed_jobs', 0)]
                             if j['name'].startswith('完整飞行 '))
    return completed


def order_configurations(configurations, latest, completed, retry_first=()):
    return sorted(configurations, key=lambda key:(
        '/'.join(map(str,key)) not in retry_first,
        bool(latest.get(key, {}).get('acceptance_pass')),
        '/'.join(map(str,key)) in completed, key))


def live(pid, expected=None):
    if not pid:
        return False
    try:
        process = psutil.Process(pid)
        return process.status() != psutil.STATUS_ZOMBIE and (expected is None or
            any(Path(arg).name == expected for arg in process.cmdline()))
    except psutil.Error:
        return False


def progress_counts(manifest, jobs, data):
    reused_checks = len(set(manifest.get('checks_reused', [])))
    reused_flights = len(set(manifest.get('current_version_flights_reused', [])))
    reused_faults = len(manifest.get('current_version_faults_reused', []))
    reused = reused_checks + reused_flights + reused_faults
    completed = data.get('completed_jobs', 0)
    return dict(completed=completed+reused, total=len(jobs)+reused,
        queue_completed=completed, queue_total=len(jobs), remaining=len(jobs)-completed,
        reused_checks=reused_checks, reused_flights=reused_flights, reused_faults=reused_faults)


def historical_recheck_progress(manifest, jobs, data):
    planned = set(manifest.get('historical_rechecks_deferred', []))
    completed = set(manifest.get('current_version_flights_reused', []))
    completed.update(job['name'].removeprefix('完整飞行 ')
        for job in jobs[:data.get('completed_jobs', 0)] if job['name'].startswith('完整飞行 '))
    return dict(total=len(planned), completed=len(planned & completed), remaining=len(planned-completed))


def show(folder):
    if folder is None or not (folder/'manifest.json').exists():
        print('尚无手动验收批次。运行 ./run_acceptance.sh plan 查看范围。')
        if HANDOFF.exists():print('自动批次交接：'+json.loads(HANDOFF.read_text())['status'])
        return
    data = state(folder)
    print('验收目录：'+str(folder))
    print('状态：'+data.get('status', '尚未启动'))
    jobs = json.loads((folder/'jobs.json').read_text())
    manifest = json.loads((folder/'manifest.json').read_text())
    progress = progress_counts(manifest, jobs, data)
    print(f"验收总进度（含已复用项目）：{progress['completed']}/{progress['total']}")
    print(f"当前队列完成：{progress['queue_completed']}/{progress['queue_total']}；剩余 {progress['remaining']} 步")
    print(f"已复用：软件／接口 {progress['reused_checks']} 项，当前版本飞行 {progress['reused_flights']} 组，故障 {progress['reused_faults']} 类")
    print('验收范围：5 项软件／接口＋18 组飞行＋4 类故障＋3 项最终核验＝30 项')
    print(f'累计曾成功飞行配置（含历史版本）：{len(historical_configurations(manifest))}/18；记录保留，版本复验另计。')
    historical = historical_recheck_progress(manifest, jobs, data)
    if historical['total']:
        print(f"本批后置版本复验：{historical['completed']}/{historical['total']} 已完成；剩余 {historical['remaining']} 组。")
    next_index = data.get('completed_jobs', 0)
    if next_index < len(jobs):print('下次续跑：'+jobs[next_index]['name']+'（之前已完成步骤不重复）')
    if data.get('job_name'):print('当前／最近项目：'+data['job_name'])
    if data.get('job_log'):print('项目日志：'+data['job_log'])
    print('队列拥有进程存活：'+str(live(data.get('pid'), 'run_autonomy_validation_queue.py')))
    print('最终机器核验：'+str(folder/'audit.json'))
    print('结果对比：'+str(folder/'results/README.md'))


def wait_for_handoff():
    warned = False
    while HANDOFF.exists() and json.loads(HANDOFF.read_text()).get('status') != 'complete':
        if not warned:
            print('等待已有试验完成收尾；不会中断当前飞行。', flush=True)
            warned = True
        time.sleep(.5)


def assert_idle():
    names = {'autonomy_session.py', 'autonomy_scene.py', 'run_autonomy_validation_queue.py',
             'run_autonomy_matrix.py', 'run_autonomy_faults.py'}
    busy = []
    for process in psutil.process_iter(['pid', 'name', 'cmdline', 'status']):
        if process.info['pid'] == os.getpid() or process.info['status'] == psutil.STATUS_ZOMBIE:
            continue
        command = process.info['cmdline'] or []
        if process.info['name'] == 'px4' or any(Path(arg).name in names for arg in command):
            busy.append(process.info['pid'])
    if busy:
        raise RuntimeError('已有仿真／验收进程运行，请等待其收尾或结束拥有它的会话：'+str(busy))


def prepare(folder, fresh, previous=None):
    folder.mkdir(parents=True, exist_ok=False)
    logs = folder/'steps';logs.mkdir()
    source = previous or {}
    source_faults = Path(source.get('faults', OLD_FAULTS))
    source_interfaces = Path(source.get('interfaces', OLD_INTERFACES))
    matrix = folder/'matrix' if fresh else Path(source.get('matrix', OLD_MATRIX))
    matrix.mkdir(parents=True, exist_ok=True)
    faults = folder/'faults';faults.mkdir()
    interfaces = folder/'interfaces'
    replay = folder/'os1_replay'
    regressions = folder/'regressions';regressions.mkdir()
    baseline = folder/'baseline_audit.json'
    code = subprocess.run([PYTHON, str(ROOT/'scripts/audit_autonomy_release.py'), '--matrix', str(matrix),
        '--faults', str(source_faults), '--interfaces', str(source_interfaces), '--output', str(baseline)],
        stdout=subprocess.DEVNULL).returncode
    if code not in (0, 1) or not baseline.exists():raise RuntimeError('无法生成初始核验清单')
    audit = json.loads(baseline.read_text())
    jobs = []
    def add(name, command, log=None):
        path = log or logs/(f'{len(jobs)+1:02d}.log')
        jobs.append(dict(name=name, command=list(map(str, command)), log_file=str(path)))
    def output_step(name, destination, command, log=None):
        add(name, [PYTHON, ROOT/'scripts/acceptance_output_step.py', '--output', destination, '--',
            *command, '--output', '{OUTPUT}'], log)
    build = Path.home()/'.cache/gazebo_sim_autonomy/build'
    for package in ('uav_autonomy', 'uav_ganyu_control'):
        if not (build/package/'CTestTestfile.cmake').exists():raise RuntimeError('请先构建验收工程：./scripts/build_autonomy.sh')
    add('任务软件回归（17 项）', ['ctest', '--test-dir', build/'uav_autonomy', '--output-on-failure'],
        regressions/'autonomy_seventeen_patrol_target.log')
    add('GanYu 软件回归（6 项）', ['ctest', '--test-dir', build/'uav_ganyu_control', '--output-on-failure'],
        regressions/'ganyu_six_clock_limits_after.log')
    add('最新尝试索引回归', ['ctest', '--test-dir', build/'uav_autonomy', '-R', '^waypoint_context_check$', '--output-on-failure'],
        regressions/'latest_fault_attempt.log')
    output_step('PointCloud2 接口（17 项）', interfaces, [PYTHON, ROOT/'scripts/cloud_interface_fixture.py'],
        regressions/'pointcloud_interface.log')
    output_step('OS1 实际 64→16 通道回放', replay, [PYTHON, ROOT/'scripts/cloud_interface_fixture.py', '--case', 'os1_sparse_replay',
        '--os1-snapshot', ROOT/'log/os1_channel_replay_20261112_verified/sparse.msg'], regressions/'os1_sparse_replay.log')
    check_jobs = list(jobs)
    reused_checks = {}
    if not fresh and source.get('interfaces'):
        origin = Path(source.get('checkpoint_directory', str(Path(source['interfaces']).parent)))
        old_checkpoint = checkpoints.read(origin)
        old_jobs_path = origin/'jobs.json'
        if old_checkpoint and old_jobs_path.is_file():
            old_jobs = {j['name']:j for j in json.loads(old_jobs_path.read_text())}
            expected_checks = checkpoints.check_contracts(signature())
            pending = []
            for job in jobs:
                name = job['name']; receipt = old_checkpoint['checks'].get(name)
                if (checkpoints.valid_receipt(receipt, expected_checks[name]) and name in old_jobs and
                        checkpoints.passed(origin, name, old_jobs[name])):
                    shutil.copy2(old_jobs[name]['log_file'], job['log_file'])
                    if name in checkpoints.CHECK_NAMES[3:]:
                        field = 'interfaces' if name == checkpoints.CHECK_NAMES[3] else 'os1_replay'
                        (interfaces if field == 'interfaces' else replay).symlink_to(Path(source[field]).resolve(), target_is_directory=True)
                    reused_checks[name] = receipt
                else:pending.append(job)
            jobs = pending
    skipped = []
    configurations = list(itertools.product(('explore', 'forest'), ('mid360', 'avia', 'os1_64'), range(3)))
    latest = latest_configurations(json.loads((matrix/'verified_results.json').read_text()))
    historical = historical_configurations(source)
    if not fresh:
        # Reproduce a known latest failure before spending time rechecking the
        # other successful configurations on their final software version.
        configurations = order_configurations(configurations, latest, historical, source.get('retry_first', []))
    for task, lidar, condition in configurations:
        label = f'{task}/{lidar}/{condition}'
        if not fresh and label in audit['verified_configurations']:
            skipped.append(label);continue
        add('完整飞行 '+label, [PYTHON, ROOT/'scripts/run_autonomy_matrix.py', '--output', matrix,
            '--resume', '--recheck-passed', '--wall-timeout', '4000', '--task', task, '--lidar', lidar,
            '--condition-index', str(condition)])
        jobs[-1]['requires_flight']=True
        if task == 'explore' and lidar == 'os1_64':jobs[-1]['min_free_gib'] = 60
    reused = []
    previous_faults = latest_fault_configurations([json.loads(p.read_text()) for p in source_faults.glob('*/fault_record.json')])
    for kind in ('cloud_pause', 'state_pause', 'manual_exit', 'blocked'):
        record = previous_faults.get((kind, 'mid360'))
        if not fresh and record and record.get('fault_check_pass') and not any(issue.startswith(kind+': ') for issue in audit['issues']):
            origin = Path(record['trial_directory'])/'fault_record.json'
            destination = faults/(kind+'_reused');destination.mkdir()
            shutil.copy2(origin, destination/'fault_record.json')
            reused.append(dict(kind=kind, source=str(origin), sha256=hashlib.sha256(origin.read_bytes()).hexdigest()))
        else:
            add('故障 '+kind, [ROOT/'run_autonomy_faults.sh', '--kind', kind, '--output', faults,
                '--wall-timeout', '1900', '--resume'])
            jobs[-1]['requires_flight']=True
    add('全部 22 份最新录包核验', [PYTHON, ROOT/'scripts/verify_closed_recording.py', '--matrix', matrix,
        '--faults', faults, '--reuse-verified', '--output', folder/'recordings.json'])
    common = ['--matrix', matrix, '--faults', faults, '--interfaces', interfaces, '--os1-replay', replay,
              '--regressions', regressions]
    add('完整交付版本机器核验', [PYTHON, ROOT/'scripts/audit_autonomy_release.py', *common, '--output', folder/'audit.json'])
    output_step('生成六组合结果与证据快照', folder/'results', [PYTHON, ROOT/'scripts/build_autonomy_results.py', *common])
    manifest = dict(created_at=datetime.now().isoformat(timespec='seconds'), fresh=fresh, matrix=str(matrix),
        faults=str(faults), interfaces=str(interfaces), os1_replay=str(replay), regressions=str(regressions),
        current_version_flights_reused=skipped, current_version_faults_reused=reused,
        implementation=signature(), checkpoint_directory=str(folder),
        checks_reused=list(reused_checks), check_jobs=check_jobs, user_review_still_required=True)
    manifest['historical_completed_configurations'] = sorted(historical)
    manifest['historical_rechecks_deferred'] = [label for label in sorted(historical) if label not in skipped]
    (folder/'manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False)+'\n')
    (folder/'jobs.json').write_text(json.dumps(jobs, indent=2, ensure_ascii=False)+'\n')
    checkpoints.write(folder, dict(execution=checkpoints.execution_contract(manifest['implementation']), checks={}))
    checkpoints.capture(folder, manifest['implementation'], [j for j in check_jobs if j['name'] in reused_checks])
    BASE.mkdir(parents=True, exist_ok=True)
    temporary = BASE/('.latest_'+str(os.getpid()));temporary.symlink_to(folder)
    os.replace(temporary, BASE/'latest')
    print(f'新批次：{folder}\n复用当前版本飞行 {len(skipped)}/18，故障 {len(reused)}/4；待执行 {len(jobs)} 个步骤。', flush=True)
    if reused_checks:print(f'复用未变化的软件／接口检查 {len(reused_checks)}/5，不重复运行。', flush=True)


def main():
    if len(sys.argv) == 1 or sys.argv[1].startswith('--') and sys.argv[1] not in ('--help', '-h'):
        sys.argv.insert(1, 'run')
    parser = argparse.ArgumentParser(description='手动启动全部验收组；默认续跑，--fresh 完整重跑。')
    commands = parser.add_subparsers(dest='action', required=True)
    for action in ('run', 'status', 'stop', 'plan'):
        sub = commands.add_parser(action)
        sub.add_argument('--output', type=Path, help='验收批次目录；省略时使用 latest')
        if action == 'run':
            mode = sub.add_mutually_exclusive_group()
            mode.add_argument('--fresh', action='store_true', help='新建独立批次，重跑全部 18 组飞行和 4 类故障')
            mode.add_argument('--new', action='store_true', help='新建续跑批次，复用当前版本已核验结果')
            sub.add_argument('--prepare-only', action='store_true', help='只生成清单和输出目录，不启动任何验收或飞机')
            sub.add_argument('--checks-only', action='store_true', help='仅运行飞行前的软件和接口检查，在第一组飞行前停止')
    args = parser.parse_args()
    folder = args.output.resolve() if args.output else (BASE/'latest').resolve() if (BASE/'latest').exists() else None
    if args.action == 'plan':
        print('正常飞行：两任务 × 三雷达 × 三初始航向，共 18 组。')
        for task, lidar in itertools.product(('explore', 'forest'), ('mid360', 'avia', 'os1_64')):
            print(f'  {task}/{lidar}：条件 0、1、2')
        print('故障：点云暂停、状态暂停、人工退出 Offboard、封路，共 4 类。')
        print('另含 17 项点云接口、OS1 稀疏回放、软件回归、22 份录包核验、版本核验和对比汇总。')
        print('30 步 = 5 步软件／接口 + 18 组正常飞行 + 4 类故障 + 3 步最终核验。')
        print('默认从本批次失败／未完成项续跑，已完成的不重复；验收工具更新只补做依赖变化的检查。')
        print('飞行程序变化时自动建立续跑清单，仅复用当前版本已核验配置；--fresh 全部重跑。')
        return
    if args.action == 'status':show(folder);return
    if args.action == 'stop':
        if folder is None or not (folder/'manifest.json').exists():raise RuntimeError('没有可停止的手动批次')
        (folder/'stop_after_current').write_text('User requested graceful stop.\n')
        print('已请求：当前验收组收尾后停止；不会中断当前飞行。');return
    runtime = Path(os.environ.get('XDG_RUNTIME_DIR') or '/tmp')
    lock = (runtime/f'gazebo_manual_acceptance_{os.getuid()}.lock').open('w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    wait_for_handoff()
    assert_idle()
    previous_path = BASE/'latest/manifest.json'
    previous = json.loads(previous_path.read_text()) if previous_path.exists() else None
    if args.fresh or args.new or folder is None:
        folder = args.output.resolve() if args.output else BASE/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid()))
        prepare(folder, args.fresh, previous)
    elif not (folder/'manifest.json').exists():
        prepare(folder, False, previous)
    else:
        implementation = signature()
        changed_checks = checkpoints.refresh_plan(folder, implementation)
        if changed_checks is None:
            manifest = json.loads((folder/'manifest.json').read_text())
            successor = BASE/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid()))
            print('飞行程序、参数或场景已变化，自动生成版本复验清单；原批次和证据保留。', flush=True)
            prepare(successor, False, manifest)
            folder = successor
    implementation = signature()
    changed_checks = checkpoints.refresh_plan(folder, implementation)
    if changed_checks:
        expected = checkpoints.check_contracts(implementation)
        reports = checkpoints.report_contract(expected)
        dependencies = {j['name']:expected[j['name']] if j['name'] in expected else reports for j in changed_checks}
        digest = hashlib.sha256(json.dumps(dependencies,sort_keys=True).encode()).hexdigest()[:16]
        update = folder/'check_updates'/digest
        attempt = 1
        while update.exists() and state(update).get('completed_jobs', 0) >= len(changed_checks):
            attempt += 1
            update = folder/'check_updates'/(digest+f'_attempt{attempt}')
        update.mkdir(parents=True, exist_ok=True)
        stop_link = update/'stop_after_current'
        if not stop_link.is_symlink():stop_link.symlink_to(folder/'stop_after_current')
        update_jobs = update/'jobs.json'
        if not update_jobs.exists():update_jobs.write_text(json.dumps(changed_checks, indent=2, ensure_ascii=False)+'\n')
        print('仅补验依赖变化或证据缺失的检查：'+', '.join(j['name'] for j in changed_checks), flush=True)
    else:update = None
    if args.prepare_only:
        print('清单已准备，尚未启动验收。再次运行 ./run_acceptance.sh 开始。')
        show(folder)
        return
    if state(folder).get('status') == 'complete' and not update:show(folder);return
    (folder/'stop_after_current').unlink(missing_ok=True)
    def stop_after_current(signum, frame):
        (folder/'stop_after_current').write_text('User interrupted foreground entry; finish current trial.\n')
        print('\n已请求当前组收尾后停止，请等待。', flush=True)
    signal.signal(signal.SIGINT, stop_after_current)
    signal.signal(signal.SIGTERM, stop_after_current)
    if update:
        update_command = [PYTHON, str(ROOT/'scripts/run_autonomy_validation_queue.py'), '--jobs', str(update/'jobs.json')]
        if (update/'state.json').exists():update_command.append('--resume')
        checker = subprocess.Popen(update_command, cwd=ROOT, start_new_session=True)
        code = checker.wait()
        completed = json.loads((update/'jobs.json').read_text())[:state(update).get('completed_jobs', 0)]
        checkpoints.capture(folder, implementation, completed)
        if code:
            print('补验未通过，主队列仍保留原进度；下次继续补验，尚未启动飞行。', flush=True)
            raise SystemExit(code)
        if (folder/'stop_after_current').exists():
            print('补验已按请求收尾停止，主队列进度保留，未启动飞行。', flush=True)
            show(folder)
            return
    command = [PYTHON, str(ROOT/'scripts/run_autonomy_validation_queue.py'), '--jobs', str(folder/'jobs.json')]
    if (folder/'state.json').exists():command.append('--resume')
    if args.checks_only:
        jobs=json.loads((folder/'jobs.json').read_text())
        command+=['--stop-before',str(nonflight_limit(jobs))]
        print('仅运行软件和接口检查，在第一组飞行前停止；不会启动飞机。',flush=True)
    else:
        print('将按清单自动启动仿真、起飞和验收；失败即停止。另一个终端可用 status 查看或 stop 请求收尾后停止。', flush=True)
    child = subprocess.Popen(command, cwd=ROOT, start_new_session=True)
    code = child.wait()
    if signature() != implementation:
        raise RuntimeError('验收期间程序发生变化；未更新检查回执，请在程序稳定后续跑。')
    checkpoints.capture(folder, implementation)
    show(folder)
    if args.checks_only and not code:
        print('非飞行检查已完成；再次运行 ./run_acceptance.sh 将继续后续验收。')
    if state(folder).get('status') == 'complete':
        print('机器验收已结束。请将此验收目录告诉我，我会继续检查结果和最终 README。')
    elif code:
        print('本批次未全部通过；保留失败证据。修复后运行同一入口续跑，或把日志目录告诉我。')
    raise SystemExit(code)


if __name__ == '__main__':
    main()
