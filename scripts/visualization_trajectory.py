"""Bounded, position-only evaluation of original EGO curves for display.

Never publishes control references and never refits or retimes a trajectory.
"""
import math
import numpy as np


def sample_native(message, max_points=600):
    if message.header.frame_id != 'odom':
        raise ValueError('Expected odom ENU trajectory')
    if not 2 <= max_points <= 2000:
        raise ValueError('Invalid visualization sample bound')
    if len(message.control_points) > 12288 or len(message.knots) > 4104 or len(message.durations) > 1024 or len(message.coefficients) > 18432:
        raise ValueError('Trajectory exceeds the display resource bound')
    if any(not math.isfinite(v) for values in (message.knots, message.control_points,
            message.durations, message.coefficients) for v in values):
        raise ValueError('Nonfinite trajectory')
    if message.representation == message.BSPLINE:
        degree = message.degree
        controls = np.asarray(message.control_points).reshape(-1, 3)
        knots = np.asarray(message.knots)
        n = len(controls)
        if not 3 <= degree <= 7 or n <= degree or len(knots) != n+degree+1 or np.any(np.diff(knots) < 0):
            raise ValueError('Invalid B-spline')
        duration = float(knots[n]-knots[degree])

        def evaluate(t):
            u = min(t+knots[degree], knots[n])
            k = min(n-1, max(degree, int(np.searchsorted(knots, u, side='right'))-1))
            values = controls[k-degree:k+1].copy()
            for r in range(1, degree+1):
                for j in range(degree, r-1, -1):
                    i = k-degree+j
                    span = knots[i+degree-r+1]-knots[i]
                    a = (u-knots[i])/span if span > 0 else 0.
                    values[j] = (1-a)*values[j-1]+a*values[j]
            return values[degree]
    elif message.representation == message.POLYNOMIAL:
        durations = np.asarray(message.durations)
        if message.degree != 5 or not len(durations) or np.any(durations <= 0) or len(message.coefficients) != 18*len(durations):
            raise ValueError('Invalid quintic polynomial')
        end = np.cumsum(durations)
        duration = float(end[-1])
        coefficients = np.asarray(message.coefficients).reshape(-1, 3, 6)

        def evaluate(t):
            i = min(len(end)-1, int(np.searchsorted(end, t, side='left')))
            local = t-(end[i-1] if i else 0.)
            value = np.zeros(3)
            for c in coefficients[i].T:
                value = value*local+c
            return value
    else:
        raise ValueError('Unknown native trajectory representation')
    if not 0 < duration <= 300:
        raise ValueError('Invalid native trajectory duration')
    times = np.linspace(0., duration, min(max_points, max(2, int(math.ceil(duration/.1))+1)))
    positions = np.asarray([evaluate(float(t)) for t in times])
    if not np.isfinite(positions).all():
        raise ValueError('Nonfinite trajectory samples')
    return times, positions
