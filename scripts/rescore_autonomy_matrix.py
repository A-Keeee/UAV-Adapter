#!/usr/bin/env python3
"""Versioned lifecycle and waypoint scoring; retain original truth and scores."""
import argparse
import csv
import hashlib
import itertools
import json
from pathlib import Path
import os

import numpy as np
from waypoint_context import patrol_truth_errors
from trial_index import latest_configurations
from evaluation_phases import acceptance_pass, enroute_metrics, enroute_samples


ROOT = Path(__file__).resolve().parents[1]
SCORE_REVISION = 3
SCORING_TOOLS = ('scripts/evaluation_phases.py', 'scripts/autonomy_evaluator.py',
                 'scripts/rescore_autonomy_matrix.py', 'scripts/waypoint_context.py')


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def scoring_tools_sha256():
    return {name: sha256(ROOT/name) for name in SCORING_TOOLS}


def _corrected_score(folder):
    """Recompute only recorded-truth metrics; never fabricate missing evidence."""
    folder = Path(folder)
    original = folder/'evaluation/evaluation.json'
    truth = folder/'evaluation/truth.csv'
    mission_path = folder/'result.json'
    score = json.loads(original.read_text())
    mission = json.loads(mission_path.read_text())
    with truth.open(newline='') as stream:
        reader = csv.DictReader(stream)
        required = {'simulation_time', 'x', 'y', 'z', 'clearance', 'controller_phase', 'task_phase'}
        if not required <= set(reader.fieldnames or []):
            raise ValueError('Truth CSV lacks lifecycle/geometry fields: '+str(truth))
        rows = list(reader)
    samples = [(float(r['simulation_time']), [float(r[a]) for a in ('x', 'y', 'z')],
                float(r['clearance']), r['controller_phase'], r['task_phase'],
                int(r.get('patrol_index', 0)), int(r.get('patrol_generation', 0))) for r in rows]
    airborne = enroute_samples(samples)
    score.update(enroute_metrics(airborne))
    task = mission.get('task')
    offset = score.get('truth_to_odom_translation')
    errors = []
    if task == 'explore':
        targets = mission.get('patrol_points_enu', [])
        scope = 'recorded_patrol_generation_and_target_index'
        if targets and offset is not None and airborne:
            if rows and 'patrol_generation' in rows[0]:
                generations = [s[6] for s in airborne]
            else:
                # Legacy trials had one automatic patrol. Do not count an
                # exploration flyby or guess repeated/manual patrol lifecycles.
                events = [json.loads(line) for line in (folder/'events.jsonl').read_text().splitlines()]
                if sum(e['event']=='exploration_ended' for e in events) != 1:
                    raise RuntimeError('Legacy patrol lifecycle is ambiguous: '+str(folder))
                starts = [s[0] for s in airborne if s[4]=='PATROL' and s[5]==0]
                generations = [int(bool(starts) and s[0]>=min(starts)) for s in airborne]
                scope = 'single_legacy_patrol_after_first_patrol_phase_and_target_index'
            errors = patrol_truth_errors([s[1] for s in airborne], [s[5] for s in airborne],
                                         generations, targets, np.asarray(offset))
        score['waypoint_context'] = scope+'_including_turn_and_replan'
        expected_targets = 4
    elif task == 'forest':
        target = mission.get('goal_enu')
        positions = np.array([s[1] for s in airborne if s[4]=='TRANSIT'])
        if target is not None and offset is not None and len(positions):
            errors = [float(np.linalg.norm(positions-(np.asarray(target)+np.asarray(offset)), axis=1).min())]
        score['waypoint_context'] = 'recorded_active_transit_goal'
        expected_targets = 1
    else:
        raise ValueError('Unknown mission task: '+str(task))
    score['waypoint_minimum_truth_errors_m'] = errors
    score['waypoints_pass'] = len(errors)>=expected_targets and all(v is not None and np.isfinite(v) and v<=.25 for v in errors)
    score['acceptance_pass'] = acceptance_pass(mission, score)
    score['score_revision'] = SCORE_REVISION
    score['correction_reason'] = ('Use the explicit takeoff/data-wait/landing lifecycle for enroute metrics, '
        'including WAIT_TAKEOFF_DATA; retain active patrol target context. All acceptance thresholds unchanged.')
    evidence = dict(original_evaluation_sha256=sha256(original), truth_csv_sha256=sha256(truth),
                    mission_result_sha256=sha256(mission_path), scoring_tools_sha256=scoring_tools_sha256())
    score['correction_evidence'] = evidence
    score['original_evaluation_sha256'] = evidence['original_evaluation_sha256']
    score['truth_csv_sha256'] = evidence['truth_csv_sha256']
    return score, original.with_name('evaluation.corrected.v3.json')


def corrected_score(folder):
    score, output = _corrected_score(folder)
    content = json.dumps(score, indent=2, allow_nan=False)+'\n'
    if output.exists():
        if output.read_text() != content:
            raise RuntimeError('Existing revision-3 correction differs; retain evidence and investigate: '+str(output))
        return score, output
    temporary = output.with_name(output.name+'.'+str(os.getpid())+'.partial')
    temporary.write_text(content)
    temporary.replace(output)
    return score, output


def correction_evidence_current(folder, score, output):
    """Verify current tools, unchanged source bytes and the recomputed result."""
    try:
        expected, destination = _corrected_score(Path(folder))
        output = Path(output)
        return bool(output.resolve()==destination.resolve() and not output.is_symlink() and
                    score.get('score_revision')==SCORE_REVISION and
                    json.dumps(score, sort_keys=True, allow_nan=False)==json.dumps(expected, sort_keys=True, allow_nan=False) and
                    json.loads(output.read_text())==score)
    except (OSError, ValueError, TypeError, KeyError, RuntimeError):
        return False


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('matrix_directory',type=Path)
    p.add_argument('--require-complete',action='store_true')
    args=p.parse_args(); results=[]
    for path in sorted(args.matrix_directory.glob('*/record.json')):
        record=json.loads(path.read_text()); folder=path.parent/'mission'
        record.setdefault('trial_directory',str(path.parent.resolve()))
        if (folder/'evaluation/evaluation.json').exists() and (folder/'result.json').exists():
            score, output=corrected_score(folder)
            record.setdefault('data',{})['evaluation']=score
            record['evaluation_file']=str(output.resolve())
            record['acceptance_pass']=bool(record.get('flight_attempted') and score.get('acceptance_pass'))
        annotation=path.parent/'development_stop.json'
        if annotation.exists():
            record['development_stop']=json.loads(annotation.read_text())
            record['reason']=record['development_stop']['reason']
            record['acceptance_pass']=False
        results.append(record)
    expected=set(itertools.product(('explore','forest'),('mid360','avia','os1_64'),range(3)))
    passed={key for key,r in latest_configurations(results).items() if r.get('acceptance_pass')}
    historically_passed={(r['task'],r['simulated_payload'],r['condition']) for r in results if r.get('acceptance_pass')}
    summary=dict(target_configurations=18,completed_records=len(results),accepted_configurations=len(passed),
        missing_or_failed=[list(k) for k in sorted(expected-passed)],complete=passed==expected,
        original_records_retained=True,waypoint_score_revision=2,score_revision=SCORE_REVISION,configuration_selection='latest_attempt',
        historically_accepted_configurations=len(historically_passed))
    (args.matrix_directory/'verified_results.json').write_text(json.dumps(results,indent=2)+'\n')
    (args.matrix_directory/'verified_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary),flush=True)
    if args.require_complete and not summary['complete']:raise SystemExit(1)


if __name__=='__main__':main()
