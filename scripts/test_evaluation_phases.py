#!/usr/bin/env python3
"""Offline lifecycle/correction regressions; no ROS nodes or simulator."""
import copy
import csv
import json
from pathlib import Path
import tempfile
import unittest

from evaluation_phases import (acceptance_pass, enroute_metrics, enroute_samples,
                               ground_contact_authorized, is_enroute)
from rescore_autonomy_matrix import corrected_score, correction_evidence_current


class EvaluationPhaseTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.folder = Path(self.temporary.name)
        (self.folder/'evaluation').mkdir()
        self.mission = dict(task='forest', success=True, reason='mission_landed', goal_enu=[2., 0., 2.])
        self.original = dict(minimum_enroute_clearance_m=.23, clearance_pass=False,
            tracking_rmse_m=.07, waypoint_minimum_truth_errors_m=[0.], waypoints_pass=True,
            truth_to_odom_translation=[0., 0., 0.], mission_completed=True,
            contact_frames=10, contact_pair_count=0, colliding_pairs=[],
            observation_cache_capacities=[400000], cache_capacity_constant=True,
            native_map_capacities=[[400000, 701784]], native_capacity_constant=True,
            coverage=None, saved_map_verified=False)
        self.rows = [
            [0., 0., 0., .23, .23, 'ACTIVE', 'WAIT_TAKEOFF_DATA', 0, 0],
            [1., 0., 0., .5, .5, 'ACTIVE', 'TAKEOFF', 0, 0],
            [2., 0., 0., 2., 2., 'ACTIVE', 'OBSERVE', 0, 0],
            [3., 1., 0., 2., 1.4, 'ACTIVE', 'TRANSIT', 0, 0],
            [4., 2., 0., 2., 1.6, 'ACTIVE', 'TRANSIT', 0, 0],
            [5., 2., 0., .23, .23, 'LANDING', 'LANDING', 0, 0]]
        self.old_corrected = self.folder/'evaluation/evaluation.corrected.json'
        self.old_corrected.write_text('{"older_revision": 2}\n')

    def write_sources(self):
        (self.folder/'result.json').write_text(json.dumps(self.mission)+'\n')
        (self.folder/'evaluation/evaluation.json').write_text(json.dumps(self.original)+'\n')
        with (self.folder/'evaluation/truth.csv').open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['simulation_time', 'x', 'y', 'z', 'clearance', 'controller_phase',
                             'task_phase', 'patrol_index', 'patrol_generation'])
            writer.writerows(self.rows)

    def score(self):
        self.write_sources()
        # Each altered fixture is an independent evidence set.
        (self.folder/'evaluation/evaluation.corrected.v3.json').unlink(missing_ok=True)
        return corrected_score(self.folder)

    def test_explicit_lifecycle(self):
        for phase in ('TAKEOFF', 'WAIT_DATA', 'WAIT_TAKEOFF_DATA', 'LANDING'):
            with self.subTest(phase=phase):
                self.assertFalse(is_enroute('ACTIVE', phase))
        for phase in ('TRANSIT', 'OBSERVE', 'WAIT_OBSERVE', 'WAIT_ROUTE', 'FAILED_HOVER'):
            with self.subTest(phase=phase):
                self.assertTrue(is_enroute('ACTIVE', phase))
        self.assertFalse(is_enroute('LANDING', 'TRANSIT'))

    def test_ground_only_contact_authorization(self):
        for phase in ('TAKEOFF', 'WAIT_TAKEOFF_DATA', 'LANDING'):
            with self.subTest(phase=phase):
                self.assertTrue(ground_contact_authorized(True, True, 'ACTIVE', phase))
                self.assertFalse(ground_contact_authorized(False, True, 'ACTIVE', phase))
        self.assertTrue(ground_contact_authorized(True, False, 'IDLE', 'WAIT_DATA'))
        self.assertFalse(ground_contact_authorized(False, False, 'IDLE', 'WAIT_DATA'))
        for phase in ('TRANSIT', 'WAIT_OBSERVE', 'WAIT_ROUTE', 'FAILED_HOVER'):
            with self.subTest(phase=phase):
                self.assertFalse(ground_contact_authorized(True, True, 'ACTIVE', phase))

    def test_corrected_forest_and_immutable_sources(self):
        self.write_sources()
        immutable = {p: p.read_bytes() for p in (self.old_corrected,
            self.folder/'result.json', self.folder/'evaluation/evaluation.json',
            self.folder/'evaluation/truth.csv')}
        score, output = corrected_score(self.folder)
        self.assertEqual(output.name, 'evaluation.corrected.v3.json')
        self.assertTrue(score['acceptance_pass'])
        self.assertEqual(score['minimum_enroute_clearance_m'], 1.4)
        self.assertEqual(score['enroute_truth_samples'], 3)
        self.assertEqual(score['path_length_m'], 2.)
        self.assertEqual(score['elapsed_airborne_sim_s'], 2.)
        self.assertTrue(correction_evidence_current(self.folder, score, output))
        self.assertEqual(len(score['correction_evidence']['scoring_tools_sha256']), 4)
        for path, before in immutable.items():
            self.assertEqual(path.read_bytes(), before)

    def test_unsafe_enroute_and_hover_stay_failed(self):
        for phase in ('TRANSIT', 'WAIT_OBSERVE', 'WAIT_ROUTE', 'FAILED_HOVER'):
            with self.subTest(phase=phase):
                self.rows[2][6] = phase
                self.rows[2][4] = .6
                score, _ = self.score()
                self.assertEqual(score['minimum_enroute_clearance_m'], .6)
                self.assertFalse(score['acceptance_pass'])

    def test_all_other_gates_remain_required(self):
        changes = [dict(contact_pair_count=1, colliding_pairs=[['drone', 'tree']]),
                   dict(contact_pair_count=0, colliding_pairs=[['drone', 'tree']]),
                   dict(contact_frames=0), dict(tracking_rmse_m=.151),
                   dict(observation_cache_capacities=[400000, 400001]),
                   dict(native_map_capacities=[[400000, 701784], [400000, 701785]]),
                   dict(cache_capacity_constant=False), dict(native_capacity_constant=False),
                   dict(mission_completed=False)]
        baseline = copy.deepcopy(self.original)
        for change in changes:
            with self.subTest(change=change):
                self.original = dict(baseline, **change)
                score, _ = self.score()
                self.assertFalse(score['acceptance_pass'])
        self.original = baseline
        self.mission['success'] = False
        self.assertFalse(self.score()[0]['acceptance_pass'])

    def test_target_error_stays_failed(self):
        self.mission['goal_enu'] = [20., 0., 2.]
        score, _ = self.score()
        self.assertFalse(score['waypoints_pass'])
        self.assertFalse(score['acceptance_pass'])

    def test_empty_and_takeoff_only_no_pass(self):
        for rows in ([], self.rows[:2]):
            with self.subTest(rows=rows):
                self.rows = rows
                score, output = self.score()
                self.assertIsNone(score['minimum_enroute_clearance_m'])
                self.assertFalse(score['acceptance_pass'])
                self.assertTrue(correction_evidence_current(self.folder, score, output))

    def test_modified_correction_rejected(self):
        score, output = self.score()
        changed = dict(score, minimum_enroute_clearance_m=3.)
        output.write_text(json.dumps(changed))
        self.assertFalse(correction_evidence_current(self.folder, score, output))
        self.assertFalse(correction_evidence_current(self.folder, changed, output))
        before = output.read_bytes()
        with self.assertRaises(RuntimeError):
            corrected_score(self.folder)
        self.assertEqual(output.read_bytes(), before)

    def test_identical_correction_is_idempotent(self):
        score, output = self.score()
        before = output.stat().st_mtime_ns
        self.assertEqual(corrected_score(self.folder), (score, output))
        self.assertEqual(output.stat().st_mtime_ns, before)

    def test_modified_source_rejected(self):
        for name in ('result.json', 'evaluation/truth.csv', 'evaluation/evaluation.json'):
            with self.subTest(name=name):
                score, output = self.score()
                with (self.folder/name).open('a') as stream:
                    stream.write('\n')
                self.assertFalse(correction_evidence_current(self.folder, score, output))

    def test_tools_hash_or_wrong_output_rejected(self):
        score, output = self.score()
        changed = copy.deepcopy(score)
        changed['correction_evidence']['scoring_tools_sha256']['scripts/evaluation_phases.py'] = '0'*64
        output.write_text(json.dumps(changed))
        self.assertFalse(correction_evidence_current(self.folder, changed, output))
        self.assertFalse(correction_evidence_current(self.folder, score, self.old_corrected))

    def test_stops_use_same_selected_lifecycle(self):
        samples = [(float(i)/10, [0., 0., 2.], 1., 'ACTIVE', 'WAIT_OBSERVE') for i in range(31)]
        samples += [(3.1, [1., 0., 2.], 1., 'ACTIVE', 'TRANSIT')]
        metrics = enroute_metrics(enroute_samples(samples))
        self.assertEqual(metrics['stops_over_2s'], 1)
        self.assertAlmostEqual(metrics['stopped_sim_s'], 3.)

    def test_indoor_patrol_generation_and_turn_context(self):
        targets = [[0., 0., 2.], [1., 0., 2.], [2., 0., 2.], [3., 0., 2.]]
        self.mission.update(task='explore', completed_patrol_points=4, patrol_points_enu=targets)
        self.original.update(coverage=.95, saved_map_verified=True)
        self.rows = [[0., 0., 0., .23, .23, 'ACTIVE', 'WAIT_TAKEOFF_DATA', 0, 0]]
        for i, target in enumerate(targets):
            self.rows.append([i+1., *target, 1.2, 'ACTIVE', 'TURN', i, 1])
        score, output = self.score()
        self.assertEqual(score['waypoint_minimum_truth_errors_m'], [0.]*4)
        self.assertTrue(score['acceptance_pass'])
        self.assertTrue(correction_evidence_current(self.folder, score, output))
        # An earlier completed patrol cannot validate the latest target set.
        self.rows.append([5., 5., 0., 2., 1.2, 'ACTIVE', 'WAIT_ROUTE', 0, 2])
        score, _ = self.score()
        self.assertEqual(score['waypoint_minimum_truth_errors_m'][1:], [None]*3)
        self.assertFalse(score['acceptance_pass'])

    def test_indoor_gates_unchanged(self):
        mission = dict(task='explore', success=True, completed_patrol_points=4)
        score = dict(self.original, mission_completed=True, minimum_enroute_clearance_m=1.,
                     coverage=.95, saved_map_verified=True)
        self.assertTrue(acceptance_pass(mission, score))
        for change in (dict(coverage=.89), dict(saved_map_verified=False), dict(coverage=None)):
            self.assertFalse(acceptance_pass(mission, dict(score, **change)))
        self.assertFalse(acceptance_pass(dict(mission, completed_patrol_points=3), score))


if __name__ == '__main__':
    unittest.main(verbosity=2)
