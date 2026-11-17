#!/usr/bin/env python3
"""Read-only qualification regressions; fixtures never start ROS or a vehicle."""
import copy
import csv
import hashlib
import json
from pathlib import Path
import tempfile
from unittest.mock import patch

from autonomy_provenance import RUNTIME_SOURCES, runtime_source_hashes
from audit_autonomy_release import task_configuration_issues
from autonomy_session import execution_height_bounds
import scoring_compatibility as compatibility
from rescore_autonomy_matrix import corrected_score, correction_evidence_current


def run():
    checks = []
    current = {name: hashlib.sha256(name.encode()).hexdigest() for name in RUNTIME_SOURCES}
    current[compatibility.EVALUATOR] = compatibility.NEW_EVALUATOR_SHA256
    old = dict(current)
    old[compatibility.EVALUATOR] = compatibility.OLD_EVALUATOR_SHA256
    delivered = {compatibility.EVALUATOR: compatibility.NEW_EVALUATOR_SHA256,
                 compatibility.PHASES: compatibility.PHASES_SHA256}
    assert compatibility.delivered_scoring_hashes() == delivered
    assert compatibility.runtime_sources_compatible(runtime_source_hashes())
    checks.append('actual_delivered_scoring_files_match_the_fixed_reviewed_hashes')
    with patch.object(compatibility, 'delivered_scoring_hashes', return_value=delivered):
        proof = compatibility.runtime_source_qualification(old, current)
        assert proof['compatible'] and proof['mode'] == 'reviewed_scoring_only_migration', proof
        assert proof['changed_recorded_runtime_sources'] == [compatibility.EVALUATOR]
        exact = compatibility.runtime_source_qualification(current, current)
        assert exact['compatible'] and exact['mode'] == 'exact_runtime_match', exact
        checks.append('reviewed_old_and_exact_new_evaluator_qualified')
        unknown = dict(old, **{compatibility.EVALUATOR: 'f'*64})
        assert not compatibility.runtime_sources_compatible(unknown, current)
        unknown_current = dict(current, **{compatibility.EVALUATOR: 'f'*64})
        assert not compatibility.runtime_sources_compatible(old, unknown_current)
        assert not compatibility.runtime_sources_compatible(unknown_current, unknown_current)
        checks.append('unknown_old_new_and_matching_unknown_scoring_revisions_rejected')
        changed = dict(current)
        changed['uav_autonomy/scripts/px4_state_bridge.py'] = 'f'*64
        assert not compatibility.runtime_sources_compatible(old, changed)
        checks.append('navigation_runtime_change_rejected')
        for malformed in (None, {}, {k:v for k,v in old.items() if k != compatibility.EVALUATOR},
                          dict(old, unexpected='f'*64), dict(old, **{compatibility.EVALUATOR:'bad'})):
            assert not compatibility.runtime_sources_compatible(malformed, current)
        checks.append('missing_extra_and_malformed_runtime_fields_rejected')
    tampered_helper = dict(delivered, **{compatibility.PHASES:'f'*64})
    with patch.object(compatibility, 'delivered_scoring_hashes', return_value=tampered_helper):
        assert not compatibility.runtime_sources_compatible(old, current)
    checks.append('changed_phase_helper_rejected')

    forest = {'autonomy':{'ros__parameters':dict(task='forest',takeoff_height=1.8,
        execution_min_z=1.5,execution_max_z=2.3,safety_radius=.65)}}
    forest_planner = {'ego_planner_node':{'ros__parameters':{
        'grid_map/virtual_ground':1.6,'grid_map/virtual_ceil':2.2}}}
    forest_run = dict(execution_z_bounds=[1.5,2.3],takeoff_height=1.8)
    original_forest = copy.deepcopy(forest)
    assert not task_configuration_issues(forest,forest,forest_planner,'v2',forest_run)
    assert forest == original_forest
    checks.append('identical_explicit_forest_bounds_pass_without_mutating_inputs')
    bad_snapshot = copy.deepcopy(forest)
    bad_snapshot['autonomy']['ros__parameters']['execution_min_z'] = 1.4
    assert task_configuration_issues(bad_snapshot,forest,forest_planner,'v2',forest_run)
    bad_manifest = dict(forest_run,execution_z_bounds=[1.4,2.3])
    assert task_configuration_issues(forest,forest,forest_planner,'v2',bad_manifest)
    bad_delivered = copy.deepcopy(forest)
    bad_delivered['autonomy']['ros__parameters']['execution_min_z'] = 1.4
    assert task_configuration_issues(forest,bad_delivered,forest_planner,'v2',forest_run)
    assert task_configuration_issues(bad_snapshot,forest,forest_planner,'v2',bad_manifest)
    checks.append('forest_snapshot_manifest_and_delivered_bounds_tampering_rejected')
    warehouse = {'autonomy':{'ros__parameters':dict(task='explore',takeoff_height=3.8,safety_radius=.65)}}
    warehouse_planner = {'ego_planner_node':{'ros__parameters':{
        'grid_map/virtual_floor_height':3.6,'grid_map/virtual_ceil_height':6.,'grid_map/resolution':.2}}}
    derived = execution_height_bounds(warehouse['autonomy']['ros__parameters'],
        warehouse_planner['ego_planner_node']['ros__parameters'],'swarm')
    warehouse_snapshot = copy.deepcopy(warehouse)
    warehouse_snapshot['autonomy']['ros__parameters'].update(execution_min_z=derived[0],execution_max_z=derived[1])
    warehouse_run = dict(execution_z_bounds=derived,takeoff_height=3.8)
    assert not task_configuration_issues(warehouse_snapshot,warehouse,warehouse_planner,'swarm',warehouse_run)
    assert task_configuration_issues(warehouse,warehouse,warehouse_planner,'swarm',warehouse_run)
    checks.append('warehouse_derived_bounds_verified_and_missing_snapshot_bounds_rejected')
    changed_planner = copy.deepcopy(warehouse_planner)
    changed_planner['ego_planner_node']['ros__parameters']['grid_map/virtual_ceil_height'] = 6.2
    assert task_configuration_issues(warehouse_snapshot,warehouse,changed_planner,'swarm',warehouse_run)
    changed_snapshot = copy.deepcopy(warehouse_snapshot)
    changed_snapshot['autonomy']['ros__parameters']['execution_max_z'] += .2
    changed_run = dict(warehouse_run,execution_z_bounds=[derived[0],derived[1]+.2])
    assert task_configuration_issues(changed_snapshot,warehouse,warehouse_planner,'swarm',warehouse_run)
    assert task_configuration_issues(warehouse_snapshot,warehouse,warehouse_planner,'swarm',changed_run)
    assert task_configuration_issues(changed_snapshot,warehouse,warehouse_planner,'swarm',changed_run)
    checks.append('warehouse_snapshot_manifest_and_planner_derived_bounds_tampering_rejected')
    bad_parameters = copy.deepcopy(forest)
    bad_parameters['autonomy']['ros__parameters']['safety_radius'] = .55
    assert task_configuration_issues(bad_parameters,forest,forest_planner,'v2',forest_run)
    assert task_configuration_issues(forest,forest,forest_planner,'v2',dict(forest_run,takeoff_height=1.7))
    checks.append('remaining_parameters_and_takeoff_manifest_still_strict')

    with tempfile.TemporaryDirectory(prefix='scoring_qualification_') as temporary:
        folder = Path(temporary)/'mission'; evaluation = folder/'evaluation'
        evaluation.mkdir(parents=True)
        original = evaluation/'evaluation.json'
        mission = folder/'result.json'
        truth = evaluation/'truth.csv'
        mission.write_text(json.dumps(dict(task='forest',success=True,goal_enu=[1.,0.,1.8])))
        original.write_text(json.dumps(dict(minimum_enroute_clearance_m=.23,clearance_pass=False,
            acceptance_pass=False,mission_completed=True,contact_frames=10,contact_pair_count=0,
            tracking_rmse_m=.07,waypoint_minimum_truth_errors_m=[.1],waypoints_pass=True,
            native_capacity_constant=True,cache_capacity_constant=True,
            native_map_capacities=[[400000,701784]],observation_cache_capacities=[400000],
            truth_to_odom_translation=[0.,0.,0.])))
        with truth.open('w',newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['simulation_time','x','y','z','clearance','controller_phase','task_phase',
                             'patrol_index','patrol_generation'])
            writer.writerow([0.,0.,0.,.23,.23,'ACTIVE','WAIT_TAKEOFF_DATA',0,0])
            writer.writerow([1.,0.,0.,1.8,1.8,'ACTIVE','TRANSIT',0,0])
            writer.writerow([2.,1.,0.,1.8,1.5,'ACTIVE','TRANSIT',0,0])
        preserved = {p:p.read_bytes() for p in (original,mission,truth)}
        score, output = corrected_score(folder)
        assert correction_evidence_current(folder,score,output)
        assert score['score_revision'] == 3 and score['minimum_enroute_clearance_m'] == 1.5
        assert score['acceptance_pass']
        assert all(p.read_bytes() == value for p,value in preserved.items())
        checks.append('version3_correction_retains_original_truth_mission_and_score')
        truth.write_bytes(preserved[truth]+b'\n')
        assert not correction_evidence_current(folder,score,output)
        truth.write_bytes(preserved[truth])
        mission.write_bytes(preserved[mission]+b'\n')
        assert not correction_evidence_current(folder,score,output)
        mission.write_bytes(preserved[mission])
        original.write_bytes(preserved[original]+b'\n')
        assert not correction_evidence_current(folder,score,output)
        original.write_bytes(preserved[original])
        checks.append('truth_mission_and_original_score_tampering_rejected')
        damaged = copy.deepcopy(score)
        damaged['correction_evidence'].pop('truth_csv_sha256')
        assert not correction_evidence_current(folder,damaged,output)
        damaged = copy.deepcopy(score)
        damaged['correction_evidence']['scoring_tools_sha256']['scripts/autonomy_evaluator.py'] = 'f'*64
        assert not correction_evidence_current(folder,damaged,output)
        assert not correction_evidence_current(folder,score,original)
        checks.append('incomplete_changed_tools_and_wrong_score_path_rejected')
        output.write_bytes(output.read_bytes()+b'\n')
        # JSON whitespace alone does not alter the score. A semantic change must fail.
        damaged = json.loads(output.read_text()); damaged['acceptance_pass'] = False
        output.write_text(json.dumps(damaged))
        assert not correction_evidence_current(folder,score,output)
        assert not correction_evidence_current(folder,damaged,output)
        checks.append('corrected_score_content_tampering_rejected')
    print(json.dumps(dict(passed=True,checks=checks),ensure_ascii=False))


if __name__ == '__main__':
    run()
