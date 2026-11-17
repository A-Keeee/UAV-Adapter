#!/usr/bin/env python3
"""Exercise manual queue controls and planning without launching ROS/flight."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from unittest.mock import patch

import manual_acceptance as entry

ROOT = Path(__file__).resolve().parents[1]
PYTHON = sys.executable
checks = []


def wait_for(predicate):
    end = time.monotonic()+10
    while time.monotonic() < end:
        if predicate():return
        time.sleep(.02)
    raise RuntimeError('Smoke check timed out')


with tempfile.TemporaryDirectory(prefix='manual_acceptance_check_') as temporary:
    base = Path(temporary)
    queue = base/'queue';queue.mkdir()
    first = queue/'first';second = queue/'second'
    jobs = [dict(name='first', log_file=str(queue/'first.log'), command=[PYTHON, '-c',
        'from pathlib import Path;import sys,time;p=Path(sys.argv[1]);p.write_text("started");time.sleep(1);p.write_text("finished");print("first finished")', str(first)]),
        dict(name='second', log_file=str(queue/'second.log'), command=[PYTHON, '-c',
        'from pathlib import Path;import sys;Path(sys.argv[1]).write_text("finished")', str(second)])]
    (queue/'jobs.json').write_text(json.dumps(jobs))
    command = [PYTHON, str(ROOT/'scripts/run_autonomy_validation_queue.py'), '--jobs', str(queue/'jobs.json')]
    child = subprocess.Popen(command, stdout=subprocess.DEVNULL)
    try:
        wait_for(first.exists)
        (queue/'stop_after_current').write_text('stop')
        assert child.wait(timeout=10) == 0
        status = json.loads((queue/'state.json').read_text())
        assert status['status'] == 'stopped_after_current' and status['completed_jobs'] == 1
        assert first.read_text() == 'finished' and not second.exists()
        checks.append('graceful_stop_finishes_current_job_and_prevents_next')
        before = first.stat().st_mtime_ns
        (queue/'stop_after_current').unlink()
        subprocess.run([*command, '--resume'], check=True, stdout=subprocess.DEVNULL)
        assert second.exists() and first.stat().st_mtime_ns == before
        assert json.loads((queue/'state.json').read_text())['status'] == 'complete'
        checks.append('resume_preserves_completed_job')
    finally:
        if child.poll() is None:child.terminate();child.wait(timeout=10)
    limited=base/'limited';limited.mkdir()
    precheck=limited/'precheck';flight=limited/'flight'
    limited_jobs=[dict(name=name,command=[PYTHON,'-c',
        'from pathlib import Path;import sys;Path(sys.argv[1]).touch()',str(marker)])
        for name,marker in [('precheck',precheck),('flight',flight)]]
    (limited/'jobs.json').write_text(json.dumps(limited_jobs))
    limited_command=[PYTHON,str(ROOT/'scripts/run_autonomy_validation_queue.py'),'--jobs',str(limited/'jobs.json')]
    subprocess.run([*limited_command,'--stop-before','0'],check=True,stdout=subprocess.DEVNULL)
    assert not precheck.exists() and not flight.exists()
    assert json.loads((limited/'state.json').read_text())['completed_jobs']==0
    subprocess.run([*limited_command,'--resume','--stop-before','1'],check=True,stdout=subprocess.DEVNULL)
    assert precheck.exists() and not flight.exists()
    limited_state=json.loads((limited/'state.json').read_text())
    assert limited_state['status']=='stopped_at_limit' and limited_state['completed_jobs']==1
    checks.append('checks_only_never_starts_the_first_flight_job')
    before=precheck.stat().st_mtime_ns
    subprocess.run([*limited_command,'--resume'],check=True,stdout=subprocess.DEVNULL)
    assert flight.exists() and precheck.stat().st_mtime_ns==before
    checks.append('normal_resume_continues_after_completed_prechecks')
    retry = base/'retry';retry.mkdir();flag = retry/'attempted'
    jobs = [dict(name='retry', log_file=str(retry/'latest.log'), command=[PYTHON, '-c',
        'from pathlib import Path;import sys;p=Path(sys.argv[1]);old=p.exists();p.touch();print("passed" if old else "failed");sys.exit(0 if old else 1)', str(flag)])]
    (retry/'jobs.json').write_text(json.dumps(jobs))
    command = [PYTHON, str(ROOT/'scripts/run_autonomy_validation_queue.py'), '--jobs', str(retry/'jobs.json')]
    assert subprocess.run(command, stdout=subprocess.DEVNULL).returncode == 1
    subprocess.run([*command, '--resume'], check=True, stdout=subprocess.DEVNULL)
    assert (retry/'latest.log').read_text().strip() == 'passed'
    assert (retry/'latest.log.attempt1').read_text().strip() == 'failed'
    checks.append('retry_preserves_failed_log_and_keeps_latest_log_separate')
    alias = base/'fixture'
    wrapper = [PYTHON, str(ROOT/'scripts/acceptance_output_step.py'), '--output', str(alias), '--', PYTHON, '-c',
        'from pathlib import Path;import sys;p=Path(sys.argv[1]);p.mkdir();(p/"result").write_text(sys.argv[2]);sys.exit(int(sys.argv[2]))', '{OUTPUT}']
    assert subprocess.run([*wrapper, '1']).returncode == 1
    failed = alias.resolve()
    subprocess.run([*wrapper, '0'], check=True)
    assert alias.resolve() != failed and (failed/'result').read_text() == '1' and (alias/'result').read_text() == '0'
    checks.append('fixture_retry_keeps_failed_output_and_publishes_latest_attempt')
    entry.BASE = base/'plans'
    fresh = entry.BASE/'fresh'
    entry.prepare(fresh, True)
    jobs = json.loads((fresh/'jobs.json').read_text())
    assert len([j for j in jobs if j['name'].startswith('完整飞行 ')]) == 18
    assert len([j for j in jobs if j['name'].startswith('故障 ')]) == 4
    assert len(jobs) == 30
    assert entry.nonflight_limit(jobs)==5 and all(not j.get('requires_flight') for j in jobs[:5])
    assert sum(bool(j.get('requires_flight')) for j in jobs)==22
    assert sum(j.get('min_free_gib')==60 for j in jobs) == 3
    assert not (fresh/'state.json').exists()
    assert not (fresh/'interfaces').exists() and not list((fresh/'matrix').glob('*/record.json'))
    checks.append('fresh_plan_covers_all_groups_without_launching_them')
    continuation = entry.BASE/'continuation'
    entry.prepare(continuation, False)
    manifest = json.loads((continuation/'manifest.json').read_text())
    audit = json.loads((continuation/'baseline_audit.json').read_text())
    jobs = json.loads((continuation/'jobs.json').read_text())
    assert len([j for j in jobs if j['name'].startswith('完整飞行 ')])+len(audit['verified_configurations']) == 18
    assert set(manifest['current_version_flights_reused']) == set(audit['verified_configurations'])
    assert all(Path(j['log_file']).parent.exists() for j in jobs)
    assert not (continuation/'state.json').exists()
    checks.append('continuation_skips_only_current_version_verified_flights')
    # Exercise the public entry, with harmless counters in place of checks and
    # flight. A changed test must not reset an already completed flight prefix.
    checkpoint = base/'checkpoint'; checkpoint.mkdir()
    counter_a = checkpoint/'counter_a'; counter_b = checkpoint/'counter_b'; airborne = checkpoint/'airborne'
    force_failure = checkpoint/'force_failure'
    dependency_a = checkpoint/'test_a.py'; dependency_b = checkpoint/'test_b.py'
    dependency_a.write_text('# first\n'); dependency_b.write_text('# first\n')
    names = entry.checkpoints.CHECK_NAMES
    prefix_jobs = [dict(name=name, log_file=str(checkpoint/f'{index}.log'), command=[PYTHON, '-c',
        'from pathlib import Path;import sys;p=Path(sys.argv[1]);p.write_text(str(int(p.read_text())+1) if p.exists() else "1");print(sys.argv[2]);sys.exit(int(Path(sys.argv[3]).exists()))',
        str(counter), f'100% tests passed, 0 tests failed out of {count}', str(force_failure)])
        for index, (name, counter, count) in enumerate([(names[0],counter_a,17),(names[1],counter_b,6)])]
    checkpoint_jobs = [*prefix_jobs, dict(name='flight sentinel', requires_flight=True,
        command=[PYTHON,'-c','from pathlib import Path;import sys;Path(sys.argv[1]).touch()',str(airborne)])]
    (checkpoint/'jobs.json').write_text(json.dumps(checkpoint_jobs))
    (checkpoint/'manifest.json').write_text(json.dumps(dict(implementation={})))
    checkpoint_command = [PYTHON,str(ROOT/'scripts/run_autonomy_validation_queue.py'),'--jobs',str(checkpoint/'jobs.json')]
    subprocess.run([*checkpoint_command,'--stop-before','2'],check=True,stdout=subprocess.DEVNULL)
    def check_dependencies(_):
        return {names[0]:entry.checkpoints.hashes([dependency_a]), names[1]:entry.checkpoints.hashes([dependency_b])}
    with patch.object(entry,'signature',return_value={}), \
            patch.object(entry.checkpoints,'execution_contract',return_value={'runtime':'unchanged'}), \
            patch.object(entry.checkpoints,'check_contracts',side_effect=check_dependencies), \
            patch.dict(os.environ,{'XDG_RUNTIME_DIR':str(checkpoint)}), \
            patch.object(entry,'wait_for_handoff'), patch.object(entry,'assert_idle'):
        entry.checkpoints.write(checkpoint,dict(execution={'runtime':'unchanged'},checks={}))
        entry.checkpoints.capture(checkpoint,{})
        assert entry.checkpoints.refresh_plan(checkpoint,{}) == []
        # Updating an unrelated tool does not invalidate the unchanged evidence.
        (checkpoint/'report_tool.py').write_text('# new report\n')
        assert entry.checkpoints.refresh_plan(checkpoint,{}) == []
        checks.append('tool_only_change_preserves_completed_checks_and_flights')
        dependency_a.write_text('# changed test\n')
        assert [j['name'] for j in entry.checkpoints.refresh_plan(checkpoint,{})] == [names[0]]
        force_failure.touch()
        with patch.object(sys,'argv',['manual_acceptance.py','--output',str(checkpoint),'--checks-only']):
            try:entry.main()
            except SystemExit as error:assert error.code == 1
            else:raise AssertionError('Failed targeted check was ignored')
        assert entry.state(checkpoint)['completed_jobs']==2 and not airborne.exists()
        assert [j['name'] for j in entry.checkpoints.refresh_plan(checkpoint,{})] == [names[0]]
        checks.append('targeted_check_failure_preserves_primary_progress_and_blocks_flight')
        force_failure.unlink()
        with patch.object(sys,'argv',['manual_acceptance.py','--output',str(checkpoint),'--checks-only']):
            try:entry.main()
            except SystemExit as error:assert error.code == 0
        assert counter_a.read_text() == '3' and counter_b.read_text() == '1' and not airborne.exists()
        assert entry.state(checkpoint)['completed_jobs']==2
        assert entry.checkpoints.refresh_plan(checkpoint,{}) == []
        checks.append('changed_check_retests_only_itself_without_resetting_primary_queue')
        # Reusing the same dependency version must not reuse a completed repair
        # queue when its evidence was subsequently removed or corrupted.
        Path(prefix_jobs[0]['log_file']).write_text('damaged after repair')
        with patch.object(sys,'argv',['manual_acceptance.py','--output',str(checkpoint),'--checks-only']):
            try:entry.main()
            except SystemExit as error:assert error.code == 0
        assert counter_a.read_text() == '4' and counter_b.read_text() == '1' and not airborne.exists()
        assert entry.checkpoints.refresh_plan(checkpoint,{}) == []
        checks.append('corrupted_repaired_evidence_starts_a_new_repair_attempt')
        # Completed job logs are evidence, not merely a counter in state.json.
        Path(prefix_jobs[1]['log_file']).write_text('corrupted evidence')
        assert [j['name'] for j in entry.checkpoints.refresh_plan(checkpoint,{})] == [names[1]]
        checks.append('missing_or_modified_evidence_requires_targeted_recheck')
        with patch.object(sys,'argv',['manual_acceptance.py','--output',str(checkpoint)]):
            try:entry.main()
            except SystemExit as error:assert error.code == 0
        assert counter_a.read_text()=='4' and counter_b.read_text()=='2' and airborne.exists()
        checks.append('normal_entry_resumes_pending_job_after_targeted_checks')
        with patch.object(entry.checkpoints,'execution_contract',return_value={'runtime':'changed'}):
            assert entry.checkpoints.refresh_plan(checkpoint,{}) is None
        checks.append('runtime_change_cannot_reuse_the_old_flight_checkpoint')
    contract = entry.checkpoints.execution_contract(entry.signature())
    assert str(ROOT/'uav_autonomy/src/autonomy_node.cpp') in contract['sources']
    assert str(ROOT/'uav_autonomy/config/warehouse.yaml') in contract['sources']
    assert str(ROOT/'scripts/autonomy_session.py') in contract['sources']
    assert str(ROOT/'scripts/manual_acceptance.py') not in contract['sources']
    assert str(ROOT/'uav_autonomy/test/waypoint_context_check.py') not in contract['sources']
    checks.append('flight_contract_includes_sources_configurations_binaries_but_excludes_test_tools')
    failed=('explore','os1_64',0); pending=('forest','mid360',0); historical=('explore','avia',0)
    ordered=entry.order_configurations([historical,pending,failed],
        {key:{'acceptance_pass':True} for key in [historical,pending,failed]},
        {'explore/avia/0'},['explore/os1_64/0'])
    assert ordered==[failed,pending,historical]
    checks.append('pending_groups_precede_historical_version_rechecks_with_explicit_failure_first')
    progress=entry.progress_counts(dict(checks_reused=list(entry.checkpoints.CHECK_NAMES),
        historical_rechecks_deferred=['explore/avia/0']*6),[{}]*25,dict(completed_jobs=0))
    assert (progress['completed'],progress['total'],progress['queue_completed'],progress['queue_total'])==(5,30,0,25)
    resumed=entry.progress_counts(dict(checks_reused=list(entry.checkpoints.CHECK_NAMES)),[{}]*25,dict(completed_jobs=1))
    assert (resumed['completed'],resumed['total'],resumed['remaining'])==(6,30,24)
    checks.append('total_progress_counts_reused_checks_without_counting_historical_flights')
    deferred = dict(historical_rechecks_deferred=['explore/avia/0','forest/mid360/0'])
    deferred_jobs = [dict(name='完整飞行 explore/avia/0'),dict(name='完整飞行 forest/mid360/0'),dict(name='final audit')]
    assert entry.historical_recheck_progress(deferred,deferred_jobs,dict(completed_jobs=0)) == dict(total=2,completed=0,remaining=2)
    assert entry.historical_recheck_progress(deferred,deferred_jobs,dict(completed_jobs=1)) == dict(total=2,completed=1,remaining=1)
    assert entry.historical_recheck_progress(deferred,deferred_jobs,dict(completed_jobs=3)) == dict(total=2,completed=2,remaining=0)
    checks.append('historical_recheck_display_uses_completed_flight_prefix')
    reused_deferred = dict(deferred,current_version_flights_reused=['forest/mid360/0'])
    assert entry.historical_recheck_progress(reused_deferred,deferred_jobs,dict(completed_jobs=1)) == dict(total=2,completed=2,remaining=0)
    checks.append('historical_recheck_display_counts_verified_reuse_without_double_counting')
print(json.dumps(dict(passed=True, checks=checks), ensure_ascii=False))
