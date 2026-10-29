"""ROS-independent 3D voxel A* and clamped degree-seven B-spline planning.

Static box collisions come from the SDF actually used by the scene. The drone is
represented by a conservative radius. Every spline span is certified using
subdivided Bezier control hulls: the convex-hull property covers the entire curve,
including portions between visualization samples. No SciPy dependency.
"""
from dataclasses import dataclass
import heapq
import itertools
import math
import xml.etree.ElementTree as ET

import numpy as np


@dataclass
class Box:
    name: str
    low: np.ndarray
    high: np.ndarray


class World:
    def __init__(self, boxes, bounds, radius=0.65, resolution=0.4):
        self.boxes = boxes
        self.bounds = np.asarray(bounds, dtype=float).reshape(2, 3)
        self.radius = float(radius)
        self.resolution = float(resolution)
        if (not np.isfinite(self.bounds).all() or np.any(self.bounds[1] <= self.bounds[0])
                or not math.isfinite(radius) or radius <= 0
                or not math.isfinite(resolution) or resolution <= 0):
            raise ValueError('Invalid map bounds, radius or resolution')
        self.shape = np.floor((self.bounds[1]-self.bounds[0])/resolution).astype(int)+1
        if np.prod(self.shape) > 2_000_000:
            raise ValueError('Map exceeds 2 million voxels')
        self.low = np.array([b.low for b in boxes], dtype=float).reshape(-1, 3)
        self.high = np.array([b.high for b in boxes], dtype=float).reshape(-1, 3)

    @classmethod
    def from_sdf(cls, filename, bounds, radius=0.65, resolution=0.4):
        world = ET.parse(filename).getroot().find('world')
        if world is None:
            raise ValueError('SDF must contain a world')
        boxes = []
        for model in world.findall('model'):
            if model.findtext('static', 'false').strip() not in ('true', '1'):
                continue
            for link in model.findall('link'):
                for collision in link.findall('collision'):
                    geometry = collision.find('geometry')
                    box = geometry.find('box') if geometry is not None else None
                    if box is None:
                        # Ground is represented by map minimum altitude; pad is visual only.
                        if geometry is not None and geometry.find('plane') is not None:
                            continue
                        raise ValueError(f'Unsupported static collision in {model.get("name")}')
                    center = np.zeros(3)
                    for element in (model, link, collision):
                        pose_element = element.find('pose')
                        if pose_element is not None and pose_element.get('relative_to'):
                            raise ValueError('relative_to SDF poses are not supported')
                        pose = np.fromstring(element.findtext('pose', '0 0 0 0 0 0'), sep=' ')
                        if pose.size != 6 or not np.isfinite(pose).all() or np.any(np.abs(pose[3:]) > 1e-8):
                            raise ValueError('Map loader requires finite, axis-aligned box poses')
                        center += pose[:3]
                    size = np.fromstring(box.findtext('size', ''), sep=' ')
                    if size.size != 3 or not np.isfinite(size).all() or np.any(size <= 0):
                        raise ValueError('Invalid SDF box size')
                    boxes.append(Box(model.get('name', 'box'), center-size/2, center+size/2))
        if not boxes:
            raise ValueError('No static obstacle boxes found')
        if world.findall('include'):
            raise ValueError('Included SDF obstacles must be expanded before planning')
        return cls(boxes, bounds, radius, resolution)

    def free(self, point, extra=0.0):
        p = np.asarray(point, dtype=float)
        return (p.shape == (3,) and np.isfinite(p).all()
                and np.all(p >= self.bounds[0]) and np.all(p <= self.bounds[1])
                and not np.any(np.all(p >= self.low-self.radius-extra, axis=1)
                               & np.all(p <= self.high+self.radius+extra, axis=1)))

    def hull_free(self, points):
        low, high = np.min(points, axis=0), np.max(points, axis=0)
        epsilon = 1e-7  # cushion for floating point Bezier conversion
        return (np.all(low >= self.bounds[0]) and np.all(high <= self.bounds[1])
                and not np.any(np.all(high >= self.low-self.radius-epsilon, axis=1)
                               & np.all(low <= self.high+self.radius+epsilon, axis=1)))

    def segment_free(self, a, b, extra=0.0):
        # Exact line/AABB slab intersection, including contact as collision.
        a, b = np.asarray(a), np.asarray(b)
        if not self.free(a, extra) or not self.free(b, extra):
            return False
        delta = b-a
        for low, high in zip(self.low-self.radius-extra, self.high+self.radius+extra):
            near, far = 0.0, 1.0
            for axis in range(3):
                if abs(delta[axis]) < 1e-12:
                    if a[axis] < low[axis] or a[axis] > high[axis]:
                        near, far = 1.0, 0.0
                        break
                else:
                    u, v = sorted(((low[axis]-a[axis])/delta[axis], (high[axis]-a[axis])/delta[axis]))
                    near, far = max(near, u), min(far, v)
            if near <= far:
                return False
        return True

    def clearance(self, point):
        if not len(self.boxes):
            return float('inf')
        p = np.asarray(point)
        return float(np.min(np.linalg.norm(np.maximum(np.maximum(self.low-p, p-self.high), 0), axis=1)))

    def astar(self, start, goal):
        start, goal = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
        if not self.free(start) or not self.free(goal):
            raise ValueError(f'Waypoint inside inflated obstacle or outside map: {start} -> {goal}')
        # Cell centers are occupied conservatively, so diagonals cannot cut corners.
        axes = [self.bounds[0, i]+np.arange(self.shape[i])*self.resolution for i in range(3)]
        xyz = np.stack(np.meshgrid(*axes, indexing='ij'), axis=-1)
        blocked = np.zeros(tuple(self.shape), dtype=bool)
        padding = self.radius + self.resolution*math.sqrt(3)/2
        for low, high in zip(self.low-padding, self.high+padding):
            blocked |= np.all((xyz >= low) & (xyz <= high), axis=-1)

        def point(index):
            return self.bounds[0] + np.asarray(index)*self.resolution

        def endpoint(p):
            center = np.rint((p-self.bounds[0])/self.resolution).astype(int)
            candidates = []
            for offset in itertools.product(range(-2, 3), repeat=3):
                index = tuple(center+offset)
                if any(index[i] < 0 or index[i] >= self.shape[i] for i in range(3)) or blocked[index]:
                    continue
                q = point(index)
                if self.segment_free(p, q):
                    candidates.append((float(np.linalg.norm(q-p)), index))
            if not candidates:
                raise ValueError('Waypoint cannot connect safely to voxel grid')
            return min(candidates)[1]

        source, target = endpoint(start), endpoint(goal)
        moves = [(d, math.sqrt(sum(v*v for v in d)))
                 for d in itertools.product((-1, 0, 1), repeat=3) if any(d)]
        def heuristic(index):
            return math.sqrt(sum((index[i]-target[i])**2 for i in range(3)))
        queue, costs, parents, closed = [(heuristic(source), 0.0, source)], {source: 0.0}, {}, set()
        while queue:
            _, cost, current = heapq.heappop(queue)
            if current in closed:
                continue
            if current == target:
                indices = [current]
                while current in parents:
                    current = parents[current]
                    indices.append(current)
                path = [start] + [point(i) for i in reversed(indices)] + [goal]
                return np.array([p for i, p in enumerate(path) if i == 0 or np.linalg.norm(p-path[i-1]) > 1e-8])
            closed.add(current)
            for delta, length in moves:
                neighbor = tuple(current[i]+delta[i] for i in range(3))
                if (any(neighbor[i] < 0 or neighbor[i] >= self.shape[i] for i in range(3))
                        or blocked[neighbor] or neighbor in closed):
                    continue
                candidate = cost+length
                if candidate < costs.get(neighbor, math.inf):
                    costs[neighbor], parents[neighbor] = candidate, current
                    heapq.heappush(queue, (candidate+heuristic(neighbor), candidate, neighbor))
        raise ValueError('A* found no collision-free route')


class Spline:
    def __init__(self, points, knots, degree=7):
        self.degree = degree
        self.points = np.asarray(points, dtype=float)
        self.knots = np.asarray(knots, dtype=float)
        self.polygons = [(self.points, self.knots, degree)]
        for _ in range(3):
            p, k, d = self.polygons[-1]
            denominators = k[d+1:len(p)+d]-k[1:len(p)]
            self.polygons.append((d*np.diff(p, axis=0)/denominators[:, None], k[1:-1], d-1))

    @property
    def duration(self):
        return float(self.knots[len(self.points)])

    def sample(self, time, order=0):
        p, k, degree = self.polygons[order]
        t = float(np.clip(time, 0, self.duration))
        span = min(len(p)-1, max(degree, int(np.searchsorted(k, t, side='right')-1)))
        d = p[span-degree:span+1].copy()
        for r in range(1, degree+1):
            for j in range(degree, r-1, -1):
                i = span-degree+j
                denominator = k[i+degree-r+1]-k[i]
                alpha = (t-k[i])/denominator if denominator else 0.0
                d[j] = (1-alpha)*d[j-1]+alpha*d[j]
        return d[degree]

    def bounds(self):
        return [float(np.max(np.linalg.norm(p, axis=1))) for p, _, _ in self.polygons[1:]]

    def certified(self, world):
        # On each knot span the spline is a degree-seven polynomial. Recover
        # its Bernstein controls, then certify tighter hulls by de Casteljau.
        degree = self.degree
        u = np.linspace(0, 1, degree+1)
        basis = np.array([[math.comb(degree, j)*t**j*(1-t)**(degree-j)
                           for j in range(degree+1)] for t in u])
        def certify(controls, depth=0):
            if world.hull_free(controls):
                return True
            if depth >= 10:
                return False
            levels = [controls]
            for _ in range(degree):
                levels.append((levels[-1][:-1]+levels[-1][1:])/2)
            left = np.array([level[0] for level in levels])
            right = np.array([level[-1] for level in reversed(levels)])
            return certify(left, depth+1) and certify(right, depth+1)
        for i in range(degree, len(self.points)):
            a, b = self.knots[i:i+2]
            samples = np.array([self.sample(a+(b-a)*t) for t in u])
            if not certify(np.linalg.solve(basis, samples)):
                return False
        return True


def plan(world, start, goal, max_velocity=1.0, max_acceleration=0.7, max_jerk=1.5):
    limits = (max_velocity, max_acceleration, max_jerk)
    if any(not math.isfinite(v) or v <= 0 for v in limits):
        raise ValueError('Dynamic limits must be finite and positive')
    raw = world.astar(start, goal)
    if np.linalg.norm(np.asarray(goal)-start) < 1e-5:
        raise ValueError('Consecutive waypoints must differ')
    # Visibility reduction keeps a little extra clearance for smoothing corners.
    reduced, index = [raw[0]], 0
    while index < len(raw)-1:
        following = index+1
        for candidate in range(len(raw)-1, index, -1):
            if world.segment_free(raw[index], raw[candidate], extra=0.1):
                following = candidate
                break
        reduced.append(raw[following])
        index = following
    for spacing in (0.8, 0.4, 0.2, 0.1, 0.05):
        dense = [reduced[0]]
        for a, b in zip(reduced, reduced[1:]):
            count = max(1, math.ceil(np.linalg.norm(b-a)/spacing))
            dense.extend(a+(b-a)*i/count for i in range(1, count+1))
        # Four coincident endpoint controls give v=a=jerk=0 at each waypoint.
        points = np.array([dense[0]]*3 + dense + [dense[-1]]*3)
        while len(points) < 8:
            points = np.insert(points, len(points)//2, (points[0]+points[-1])/2, axis=0)
        n, degree = len(points), 7
        knots = np.array([0.0]*8 + list(range(1, n-degree)) + [float(n-degree)]*8)
        spline = Spline(points, knots)
        if not spline.certified(world):
            continue
        scale = max(0.05, *(bound/limit if order == 1 else (bound/limit)**(1/order)
                           for order, (bound, limit) in enumerate(zip(spline.bounds(), limits), 1)))
        spline = Spline(points, knots*scale*1.01)
        sampled = np.array([spline.sample(t) for t in np.linspace(0, spline.duration, max(50, n*2))])
        return raw, spline, {'astar_length_m': float(np.sum(np.linalg.norm(np.diff(raw, axis=0), axis=1))),
                            'spline_length_m': float(np.sum(np.linalg.norm(np.diff(sampled, axis=0), axis=1))),
                            'duration_s': spline.duration, 'derivative_bounds': spline.bounds(),
                            'minimum_sampled_clearance_m': min(world.clearance(p) for p in sampled),
                            'collision_certificate': 'all subdivided Bezier hull AABBs free',
                            'control_points': n}
    raise ValueError('Could not certify smoothed spline; no trajectory will be sent')
