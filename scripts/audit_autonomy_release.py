#!/usr/bin/env python3
"""Require complete current-version flight, fault and interface evidence."""
import argparse
import copy
import hashlib
import itertools
import json
from pathlib import Path
import subprocess
import sys
import yaml

from autonomy_provenance import ROOT, runtime_source_hashes
from autonomy_session import execution_height_bounds
from rescore_autonomy_matrix import correction_evidence_current
from scoring_compatibility import migration_definition, runtime_source_qualification
from trial_index import latest_configurations, latest_fault_configurations
from verify_closed_recording import current_evidence as current_recording_evidence


def task_configuration_issues(actual, delivered, planner, backend, run):
    """Normalize generated bounds only after proving all three sources agree."""
    actual=copy.deepcopy(actual);delivered=copy.deepcopy(delivered)
    actual_parameters=actual['autonomy']['ros__parameters']
    delivered_parameters=delivered['autonomy']['ros__parameters']
    expected_bounds=execution_height_bounds(delivered_parameters,
        planner['ego_planner_node']['ros__parameters'],backend)
    snapshot_bounds=[actual_parameters.pop('execution_min_z',None),
                     actual_parameters.pop('execution_max_z',None)]
    manifest_bounds=run.get('execution_z_bounds')
    issues=[]
    if snapshot_bounds!=manifest_bounds:
        issues.append('execution bounds disagree with the run manifest')
    if snapshot_bounds!=expected_bounds or manifest_bounds!=expected_bounds:
        issues.append('execution bounds differ from the delivered task/planner safety limits')
    # Both explicit task limits and launcher-derived limits are represented in
    # the execution snapshot. Their values were checked above, so normalize the
    # same two fields on each side before comparing every remaining parameter.
    delivered_parameters.pop('execution_min_z',None)
    delivered_parameters.pop('execution_max_z',None)
    if actual_parameters.get('takeoff_height')!=run.get('takeoff_height'):
        issues.append('controller takeoff height disagrees with task configuration')
    if actual!=delivered:
        issues.append('configuration differs from delivered parameters')
    return issues


def current_interface_evidence(folder):
    path=folder/'summary.json'
    if not path.exists():return False
    summary=json.loads(path.read_text());binary=ROOT/'install/uav_autonomy/lib/uav_autonomy/autonomy_node'
    return bool(summary.get('implementation_constant_during_test') and
        summary.get('runtime_sources_sha256')==runtime_source_hashes() and
        summary.get('autonomy_binary_sha256')==hashlib.sha256(binary.read_bytes()).hexdigest())


def current_os1_replay(folder):
    if not current_interface_evidence(folder) or not (folder/'results.json').exists():return False
    summary=json.loads((folder/'summary.json').read_text());selection=summary.get('recorded_os1_channel_selection') or {}
    cases=json.loads((folder/'results.json').read_text());case=next((r for r in cases if r['case']=='os1_sparse_replay'),{})
    snapshot=Path(summary.get('recorded_os1_snapshot') or '/missing')
    return bool(case.get('passed') and selection.get('interface_replay_verified') and
        selection.get('pattern_channels')==64 and selection.get('measured_channels')==64 and selection.get('retained_channels')==16 and
        selection.get('maximum_ray_distance_m',1)<.001 and selection.get('navigation_received_model_parameters') is False and
        snapshot.is_file() and hashlib.sha256(snapshot.read_bytes()).hexdigest()==selection.get('sparse_cloud_sha256') and
        case.get('diagnostics',{}).get('input_points')==selection.get('retained_points'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--matrix',required=True,type=Path)
    parser.add_argument('--faults',required=True,type=Path)
    parser.add_argument('--interfaces',required=True,type=Path)
    parser.add_argument('--os1-replay',type=Path,default=ROOT/'log/os1_channel_replay_results/latest')
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--regressions',type=Path,default=ROOT/'log/autonomy_regression_20261112/latest')
    args=parser.parse_args();issues=[];verified=[];accepted_count=0
    runtime_qualifications=[];score_evidence=[]
    subprocess.run([sys.executable,str(ROOT/'scripts/rescore_autonomy_matrix.py'),str(args.matrix)],check=True,stdout=subprocess.DEVNULL)
    records=json.loads((args.matrix/'verified_results.json').read_text())
    latest=latest_configurations(records);sources=runtime_source_hashes()
    def current_implementation(folder,label,expected_task,full_flight=True):
        manifest=folder/'run.json'
        if not manifest.exists():issues.append(label+': missing run manifest');return
        run=json.loads(manifest.read_text());software=run.get('software',{})
        qualification=runtime_source_qualification(software.get('runtime_sources_sha256'),sources)
        runtime_qualifications.append(dict(configuration=label,**qualification))
        if not qualification['compatible']:
            issues.append(label+': runtime Python sources have not been revalidated: '+qualification['reason'])
        binaries=software.get('binaries_sha256',{})
        required=[ROOT/'install/uav_autonomy/lib/uav_autonomy/autonomy_node',ROOT/'install/uav_ganyu_control/lib/uav_ganyu_control/ganyu_controller']
        backend=run.get('backend')
        if backend not in ('swarm','v2'):issues.append(label+': invalid backend');return
        if run.get('task')!=expected_task or backend!=('swarm' if expected_task=='explore' else 'v2'):
            issues.append(label+': task/backend pairing differs from the plan')
        configuration=Path(run.get('configuration_directory',str(folder/'configuration')))
        task_name='warehouse.yaml' if expected_task=='explore' else 'forest.yaml'
        delivered_planner=ROOT/'uav_autonomy/config'/f'ego_{backend}.yaml'
        planner_configuration=yaml.safe_load(delivered_planner.read_text())
        for name in (task_name,f'ego_{backend}.yaml'):
            snapshot=configuration/name;delivered=ROOT/'uav_autonomy/config'/name
            if not snapshot.is_file():issues.append(label+': configuration snapshot missing: '+name);continue
            actual=yaml.safe_load(snapshot.read_text());expected_config=yaml.safe_load(delivered.read_text())
            if name==task_name:
                issues.extend(label+': '+issue+': '+name for issue in task_configuration_issues(
                    actual,expected_config,planner_configuration,backend,run))
            elif actual!=expected_config:issues.append(label+': configuration differs from delivered parameters: '+name)
        overlay=Path.home()/f'.cache/gazebo_ego_{backend}/install'
        required += [overlay/'ego_planner/lib/ego_planner/ego_planner_node',overlay/'ego_ganyu_bridge/lib/ego_ganyu_bridge/ego_ganyu_bridge']
        required += list(overlay.glob('*/lib/*.so'))
        for path in required:
            if not path.exists() or binaries.get(str(path))!=hashlib.sha256(path.read_bytes()).hexdigest():
                issues.append(label+': binary differs: '+str(path))
        scene=run.get('scene',{});world=Path(scene.get('world_file','/missing'))
        if not world.is_file() or hashlib.sha256(world.read_bytes()).hexdigest()!=scene.get('world_sha256'):
            issues.append(label+': world snapshot is missing or changed')
        canonical=ROOT/'assets/gazebo_world'/(scene.get('world','missing')+'.sdf')
        if not canonical.is_file() or not world.is_file() or canonical.read_bytes()!=world.read_bytes():
            issues.append(label+': scenario differs from the delivered world')
        if not run.get('automatic_start') or not run.get('cloud_topic','').startswith(('/anonymous/','/fault/')):
            issues.append(label+': automatic anonymous-input flight is unproven')
        checkpoint=folder/'evaluation/evaluation_checkpoint.json'
        if not checkpoint.exists() or not json.loads(checkpoint.read_text()).get('finalized'):
            issues.append(label+': final independent contact checkpoint missing')
        if not current_recording_evidence(folder/'bag',require_flight=full_flight):
            issues.append(label+': closed recording integrity and ROS replay evidence missing or changed')
    expected=set(itertools.product(('explore','forest'),('mid360','avia','os1_64'),range(3)))
    for key in sorted(expected):
        label='/'.join(map(str,key));record=latest.get(key)
        if not record:issues.append(label+': latest full flight is not accepted');continue
        accepted_count+=int(bool(record.get('acceptance_pass')));before=len(issues)
        folder=Path(record['trial_directory'])/'mission'
        score=record.get('data',{}).get('evaluation',{})
        evaluation=Path(record.get('evaluation_file',folder/'evaluation/evaluation.json'))
        if not correction_evidence_current(folder,score,evaluation):
            issues.append(label+': current version-3 scoring evidence is missing or changed')
        else:
            score_evidence.append(dict(configuration=label,score_revision=score['score_revision'],
                evaluation_file=str(evaluation.resolve()),
                evaluation_sha256=hashlib.sha256(evaluation.read_bytes()).hexdigest(),
                correction_evidence=score['correction_evidence']))
        if not record.get('acceptance_pass'):
            issues.append(label+': latest full flight is not accepted');continue
        if bool(score.get('acceptance_pass')) != bool(record['acceptance_pass']):
            issues.append(label+': indexed acceptance disagrees with scoring evidence')
        current_implementation(folder,label,key[0])
        if record.get('navigation_sensor_metadata_provided') is not False:issues.append(label+': anonymous sensor requirement unproven')
        returns=folder.parent/'scene_returns.json'
        if not returns.exists() or not json.loads(returns.read_text()).get('physics_surface_echo_pass'):
            issues.append(label+': physical lidar return validation missing')
        if len(issues)==before:verified.append(label)
    fault_records=[json.loads(p.read_text()) for p in args.faults.glob('*/fault_record.json')]
    latest_faults=latest_fault_configurations(fault_records)
    for kind in ('cloud_pause','state_pause','manual_exit','blocked'):
        record=latest_faults.get((kind,'mid360'))
        if not record or not record.get('fault_check_pass'):issues.append(kind+': latest physical fault trial is not accepted');continue
        current_implementation(Path(record['trial_directory'])/'mission',kind,'forest',full_flight=False)
    interface_file=args.interfaces/'results.json'
    interface_cases=json.loads(interface_file.read_text()) if interface_file.exists() else []
    required_cases={'xyz','reordered','float64','padded','big_endian','sparse','low_frequency','cropped_forward',
        'world_with_origin','world_without_origin','empty','missing_tf','skewed_clock','mixed_xyz_types',
        'nonfinite','spatial_overload','resource_limit'}
    passed_cases={r['case'] for r in interface_cases if r.get('passed')}
    if not required_cases<=passed_cases:issues.append('Missing or failed interface cases: '+','.join(sorted(required_cases-passed_cases)))
    if not current_interface_evidence(args.interfaces):
        issues.append('Interface validation has not verified the delivered implementation')
    if not current_os1_replay(args.os1_replay):issues.append('Current-version recorded OS1-64 to 16-channel replay is unverified')
    # Software regressions remain separate from actual flight acceptance.
    regression=args.regressions
    for name,count in [('autonomy_seventeen_patrol_target.log',17),('ganyu_six_clock_limits_after.log',6),('latest_fault_attempt.log',1)]:
        path=regression/name;text=path.read_text() if path.exists() else ''
        if f'100% tests passed, 0 tests failed out of {count}' not in text:issues.append('Full regression not passed: '+name)
    result=dict(complete=not issues,accepted_normal_configurations=accepted_count,current_version_verified_configurations=len(verified),target_normal_configurations=18,
        verified_configurations=verified,required_physical_faults=4,required_interface_cases=17,required_os1_recorded_channel_replays=1,
        required_verified_closed_recordings=22,
        runtime_sources_sha256=sources,issues=issues,readme_completion_still_requires_documentation_review=True,
        scoring_migration=migration_definition(),runtime_source_qualifications=runtime_qualifications,
        versioned_score_evidence=score_evidence,
        matrix_index_sha256=hashlib.sha256((args.matrix/'verified_results.json').read_bytes()).hexdigest(),
        audit_tool_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)
    if issues:raise SystemExit(1)


if __name__=='__main__':main()
