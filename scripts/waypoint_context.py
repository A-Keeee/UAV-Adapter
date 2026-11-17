"""Truth waypoint errors scoped to a commanded patrol and its active target."""
import numpy as np


def patrol_truth_errors(positions, indices, generations, targets, offset):
    points = np.asarray(positions, dtype=float).reshape(-1, 3)
    indices = np.asarray(indices, dtype=int)
    generations = np.asarray(generations, dtype=int)
    latest = int(generations.max()) if len(generations) else 0
    errors = []
    for index, target in enumerate(targets):
        selected = points[(generations == latest) & (generations > 0) & (indices == index)]
        errors.append(float(np.linalg.norm(selected-(np.asarray(target)+offset), axis=1).min())
                      if len(selected) else None)
    return errors
