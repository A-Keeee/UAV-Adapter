"""Independent scoring rules for takeoff, enroute flight and landing.

These phases affect evaluation only. Waiting for observations after takeoff,
route repair and failure hover remain enroute and retain all safety checks.
"""
import math


NON_ENROUTE_PHASES = frozenset(('TAKEOFF', 'WAIT_DATA', 'WAIT_TAKEOFF_DATA', 'LANDING'))


def is_enroute(controller_phase, task_phase):
    return controller_phase == 'ACTIVE' and task_phase not in NON_ENROUTE_PHASES


def ground_contact_authorized(is_ground, armed, controller_phase, task_phase):
    # Only ground contact receives the existing takeoff/landing exemption.
    return bool(is_ground and (not armed or controller_phase == 'LANDING' or
                               task_phase in NON_ENROUTE_PHASES))


def enroute_samples(samples):
    """Select tuples (time, XYZ, clearance, controller, task, ...)."""
    return [s for s in samples if is_enroute(s[3], s[4])]


def enroute_metrics(samples):
    """Compute the same metrics from an already selected enroute sequence."""
    distance = sum(math.dist(b[1], a[1]) for a, b in zip(samples, samples[1:]))
    stop_count = 0
    stopped_time = 0.
    segment_start = None
    for a, b in zip(samples, samples[1:]):
        dt = b[0]-a[0]
        stopped = 0 < dt < .5 and math.dist(b[1], a[1])/dt < .05
        if stopped and segment_start is None:
            segment_start = a[0]
        if not stopped and segment_start is not None:
            duration = a[0]-segment_start
            if duration >= 2:
                stop_count += 1
                stopped_time += duration
            segment_start = None
    if segment_start is not None:
        duration = samples[-1][0]-segment_start
        if duration >= 2:
            stop_count += 1
            stopped_time += duration
    clearance = min((s[2] for s in samples), default=None)
    return dict(enroute_truth_samples=len(samples),
                path_length_m=distance,
                elapsed_airborne_sim_s=samples[-1][0]-samples[0][0] if len(samples) > 1 else 0.,
                minimum_enroute_clearance_m=clearance,
                clearance_pass=_finite(clearance) and clearance >= .65,
                stops_over_2s=stop_count, stopped_sim_s=stopped_time)


def _finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def acceptance_pass(mission, score):
    """Apply every original mission, truth, collision and bounded-map gate."""
    task = mission.get('task')
    if task not in ('forest', 'explore'):
        return False
    clearance = score.get('minimum_enroute_clearance_m')
    rmse = score.get('tracking_rmse_m')
    passed = bool(mission.get('success') and score.get('mission_completed') and score.get('waypoints_pass') and
                  _finite(clearance) and clearance >= .65 and
                  _finite(rmse) and rmse <= .15 and
                  score.get('contact_frames', 0) > 0 and
                  score.get('contact_pair_count') == 0 and
                  not score.get('colliding_pairs'))
    if task == 'explore':
        coverage = score.get('coverage')
        passed = bool(passed and mission.get('completed_patrol_points', 0) >= 4 and
                      _finite(coverage) and coverage >= .9 and
                      score.get('saved_map_verified'))
    else:
        passed = bool(passed and score.get('cache_capacity_constant') and score.get('native_capacity_constant') and
                      len(score.get('observation_cache_capacities', [])) == 1 and
                      len(score.get('native_map_capacities', [])) == 1)
    return passed
