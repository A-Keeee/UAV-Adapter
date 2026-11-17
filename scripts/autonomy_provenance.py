"""Record the exact local implementation used by each simulation trial."""
import hashlib
import json
from pathlib import Path
import subprocess
import psutil

ROOT = Path(__file__).resolve().parents[1]
RUNTIME_SOURCES = (
    'uav_autonomy/scripts/px4_state_bridge.py', 'uav_autonomy/scripts/cloud_proxy.py',
    'scripts/autonomy_session.py', 'scripts/autonomy_scene.py',
    'scripts/autonomy_evaluator.py', 'scripts/autonomy_geometry.py',
    'scripts/evaluation_finalization.py', 'scripts/owned_processes.py',
    'scripts/ego_environment.py', 'scripts/waypoint_context.py',
)


def runtime_source_hashes():
    # Keep reporting/documentation tools separate from code used by a flight.
    return {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in RUNTIME_SOURCES}


def active_scene(path):
    """Never import mass or evaluation metadata from a stale simulator run."""
    try:
        scene = json.loads(path.read_text())
        owner = psutil.Process(scene['owner_pid'])
        if abs(owner.create_time()-scene['owner_start_time'])>.01 or not owner.is_running(): return None
        if not any(Path(arg).name == 'autonomy_scene.py' for arg in owner.cmdline()): return None
        scene['scene_directory'] = str(path.parent.resolve())
        return scene
    except (OSError, ValueError, KeyError, TypeError, psutil.Error):
        return None


def fingerprint(backend):
    digest = hashlib.sha256()
    paths = []
    for name in ('uav_autonomy', 'uav_common_msg', 'uav_ganyu_control', 'third_party/ego_bridge',
                 'third_party/ego_'+backend, 'scripts'):
        for path in (ROOT/name).rglob('*'):
            if path.is_file() and path.suffix in ('.py', '.cpp', '.hpp', '.h', '.xml', '.yaml', '.txt', '.sh'):
                if '__pycache__' not in path.parts: paths.append(path)
    for path in sorted(paths):
        digest.update(str(path.relative_to(ROOT)).encode()); digest.update(b'\0'); digest.update(path.read_bytes())
    binaries = [ROOT/'install/uav_autonomy/lib/uav_autonomy/autonomy_node',
                ROOT/'install/uav_ganyu_control/lib/uav_ganyu_control/ganyu_controller',
                Path.home()/f'.cache/gazebo_ego_{backend}/install/ego_planner/lib/ego_planner/ego_planner_node',
                Path.home()/f'.cache/gazebo_ego_{backend}/install/ego_ganyu_bridge/lib/ego_ganyu_bridge/ego_ganyu_bridge']
    binaries += list((Path.home()/f'.cache/gazebo_ego_{backend}/install').glob('*/lib/*.so'))
    return dict(source_sha256=digest.hexdigest(), files=len(paths),
                git_revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
                runtime_sources_sha256=runtime_source_hashes(),
                binaries_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in binaries if p.exists()})
