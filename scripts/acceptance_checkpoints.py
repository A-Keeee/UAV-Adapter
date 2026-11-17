"""Separate flight implementation changes from checks and reporting tools.

Receipts are tied to dependency hashes AND immutable result bytes. They never
convert a historical flight into evidence for a changed flight implementation.
"""
import ast
import hashlib
import json
import re
from pathlib import Path

from autonomy_provenance import ROOT, RUNTIME_SOURCES

EXTENSIONS = {'.py', '.cpp', '.hpp', '.h', '.xml', '.yaml', '.txt', '.sh', '.msg'}
CHECK_NAMES = ('任务软件回归（17 项）', 'GanYu 软件回归（6 项）', '最新尝试索引回归',
               'PointCloud2 接口（17 项）', 'OS1 实际 64→16 通道回放')
REPORT_NAMES = ('全部 22 份最新录包核验', '完整交付版本机器核验', '生成六组合结果与证据快照')


def hashes(paths):
    return {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(set(paths)) if p.is_file()}


def tree(name, tests=True):
    return [p for p in (ROOT/name).rglob('*') if p.is_file() and p.suffix in EXTENSIONS and
            '__pycache__' not in p.parts and (tests or 'test' not in p.relative_to(ROOT).parts)]


def python_dependencies(paths):
    """Follow local imports; external ROS/Python packages are not source files here."""
    pending = list(paths); found = set()
    while pending:
        p = pending.pop()
        if p in found or not p.is_file(): continue
        found.add(p)
        if p.suffix != '.py': continue
        for node in ast.walk(ast.parse(p.read_text())):
            modules = ([node.module] if isinstance(node, ast.ImportFrom) and node.module else
                       [n.name for n in node.names] if isinstance(node, ast.Import) else [])
            for module in modules:
                for directory in (p.parent, ROOT/'scripts', ROOT/'uav_autonomy/scripts'):
                    candidate = directory/(module.replace('.', '/')+'.py')
                    if candidate.is_file(): pending.append(candidate)
    return found


def execution_contract(implementation):
    sources = []
    for name in ('uav_autonomy', 'uav_common_msg', 'uav_ganyu_control',
                 'third_party/ego_bridge', 'third_party/ego_swarm', 'third_party/ego_v2'):
        sources += tree(name, tests=False)
    entries = [ROOT/n for n in RUNTIME_SOURCES]
    entries += [ROOT/'scripts'/n for n in ('run_autonomy_matrix.py', 'run_autonomy_faults.py')]
    sources += list(python_dependencies(entries))
    sources += list(ROOT.glob('run_autonomy*.sh'))
    return dict(sources=hashes(sources), scenarios=implementation['scenarios'],
                binaries={b: s['binaries_sha256'] for b, s in implementation['software'].items()})


def check_contracts(implementation):
    binary = ROOT/'install/uav_autonomy/lib/uav_autonomy/autonomy_node'
    build = Path.home()/'.cache/gazebo_sim_autonomy/build'
    result = {}
    for key, package in enumerate(('uav_autonomy', 'uav_ganyu_control')):
        files = tree(package) + tree('uav_common_msg')
        if key == 0:
            files += tree('uav_ganyu_control') + tree('third_party/ego_bridge')
            files += list((ROOT/'uav_autonomy/test').rglob('*.json'))
            files += [Path(p) for software in implementation['software'].values()
                      for p in software['binaries_sha256'] if p.endswith(('/ego_ganyu_bridge/ego_ganyu_bridge', '/ego_planner/ego_planner_node', '.so'))]
        files += list(python_dependencies([p for p in files if p.suffix == '.py']))
        testfile = build/package/'CTestTestfile.cmake'
        files.append(testfile)
        if testfile.is_file():
            for name in re.findall(r'add_test\(\S+ "([^"]+)"', testfile.read_text()):
                path = Path(name)
                if path.is_relative_to(build): files.append(path)
        files.append(binary if key == 0 else ROOT/'install/uav_ganyu_control/lib/uav_ganyu_control/ganyu_controller')
        result[CHECK_NAMES[key]] = hashes(files)
    result[CHECK_NAMES[2]] = hashes(python_dependencies([ROOT/'uav_autonomy/test/waypoint_context_check.py']))
    fixture = [ROOT/'scripts'/n for n in ('cloud_interface_fixture.py', 'verify_cloud_interface.py',
                                       'interface_fastdds.xml', 'acceptance_output_step.py')]
    files = list(python_dependencies(fixture)) + [binary]
    common = dict(files=hashes(files), runtime_sources=implementation['software']['swarm']['runtime_sources_sha256'])
    result[CHECK_NAMES[3]] = common
    result[CHECK_NAMES[4]] = dict(**common, snapshot=hashes([ROOT/'log/os1_channel_replay_20261112_verified/sparse.msg']))
    return result


def read(folder):
    path = folder/'checkpoints.json'
    return json.loads(path.read_text()) if path.is_file() else None


def report_contract(checks):
    tools = [ROOT/'scripts'/name for name in ('verify_closed_recording.py',
        'audit_autonomy_release.py', 'build_autonomy_results.py', 'acceptance_output_step.py')]
    return dict(tools=hashes(python_dependencies(tools)), checks=checks)


def write(folder, data):
    target = folder/'checkpoints.json'; temporary = target.with_suffix('.json.partial')
    temporary.write_text(json.dumps(data, indent=2, ensure_ascii=False)+'\n'); temporary.replace(target)


def evidence_files(folder, name, job):
    manifest = json.loads((folder/'manifest.json').read_text())
    files = [Path(job['log_file'])]
    if name in CHECK_NAMES[3:]:
        output = Path(manifest['interfaces' if name == CHECK_NAMES[3] else 'os1_replay'])
        files += [output/'summary.json', output/'results.json']
    return files


def passed(folder, name, job):
    files = evidence_files(folder, name, job)
    if not all(p.is_file() for p in files): return False
    if name in CHECK_NAMES[:3]:
        count = (17, 6, 1)[CHECK_NAMES.index(name)]
        return f'100% tests passed, 0 tests failed out of {count}' in files[0].read_text()
    results = json.loads(files[-1].read_text())
    from audit_autonomy_release import current_interface_evidence, current_os1_replay
    output = files[-1].parent
    if name == CHECK_NAMES[4]: return current_os1_replay(output)
    return len(results) == 17 and all(r.get('passed') for r in results) and current_interface_evidence(output)


def capture(folder, implementation, completed_jobs=None):
    data = read(folder)
    if data is None: raise RuntimeError('Missing checkpoint dependency baseline')
    expected = check_contracts(implementation)
    jobs = json.loads((folder/'jobs.json').read_text())
    if completed_jobs is None:
        state = json.loads((folder/'state.json').read_text()) if (folder/'state.json').exists() else {}
        completed_jobs = jobs[:state.get('completed_jobs', 0)]
    for job in completed_jobs:
        name = job['name']
        if name in REPORT_NAMES:
            data.setdefault('reports', {})[name] = report_contract(expected)
        if name in expected and passed(folder, name, job):
            receipt = dict(dependencies=expected[name],
                evidence=hashes(p.resolve() for p in evidence_files(folder, name, job)))
            previous = data['checks'].get(name)
            if previous and previous != receipt:
                data.setdefault('check_history', []).append(dict(name=name, receipt=previous))
            data['checks'][name] = receipt
    write(folder, data)


def valid_receipt(receipt, expected):
    return bool(receipt and receipt['dependencies'] == expected and receipt.get('evidence') and
                hashes(map(Path, receipt['evidence'])) == receipt['evidence'])


def refresh_plan(folder, implementation):
    """Return changed completed checks, leaving the primary flight queue intact."""
    data = read(folder)
    if data is None:
        original = json.loads((folder/'manifest.json').read_text())['implementation']
        if original != implementation:
            raise RuntimeError('旧批次缺少依赖基线且程序已变化；请使用 --new，不能证明的旧检查不会复用。')
        write(folder, dict(execution=execution_contract(implementation), checks={}))
        capture(folder, implementation)
        data = read(folder)
    if data['execution'] != execution_contract(implementation): return None
    jobs = json.loads((folder/'jobs.json').read_text())
    state = json.loads((folder/'state.json').read_text()) if (folder/'state.json').exists() else {}
    expected = check_contracts(implementation)
    changed = []
    manifest = json.loads((folder/'manifest.json').read_text())
    completed = {j['name']:j for j in jobs[:state.get('completed_jobs', 0)]}
    # A successor may reuse checks instead of keeping them in its main queue.
    for job in manifest.get('check_jobs', []):
        if job['name'] in data['checks']: completed.setdefault(job['name'], job)
    for job in completed.values():
        name = job['name']
        if name not in expected: continue
        receipt = data['checks'].get(name)
        if (not valid_receipt(receipt, expected[name]) or
                hashes(p.resolve() for p in evidence_files(folder, name, job)) != receipt['evidence']):
            changed.append(job)
    report_dependencies = report_contract(expected)
    for job in jobs[:state.get('completed_jobs', 0)]:
        if job['name'] in REPORT_NAMES and data.get('reports', {}).get(job['name']) != report_dependencies:
            changed.append(job)
    return changed
