#!/usr/bin/env python3
"""Materialize pinned, offline Harmonic worlds and interchangeable RGL payloads."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import urllib.request
import zipfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
CACHE = Path.home() / '.cache/gazebo_autonomy'
FUEL = [
    ('pine.zip', 'OpenRobotics', 'Pine Tree', 6),
    ('oak.zip', 'OpenRobotics', 'Oak tree', 7),
    ('grass.zip', 'hexarotor', 'grasspatch', 1),
]
FUEL_SHA256 = {
    'pine.zip': 'd436a44229321314e79e4c2ba709dc926e833a5b5d90d16c5d085c2a74576e25',
    'oak.zip': 'ae128176f707b571d2a2a47a1ab6b80596e81d96402501a64f4f28a49543daa5',
    'grass.zip': '23d8b5eac474368b57d1bea9f831d5bf04ac39e45d7b311422434ad43b0a5d6b',
}


def write_xml(root, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    if root.tag == 'sdf' and root.find('world') is not None:
        root.set('version', '1.10')  # Custom spherical reference axes.
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def systems(world):
    template = ET.parse(ROOT / 'assets/gazebo_world/walls_demo.sdf').getroot().find('world')
    for child in template:
        if child.tag in ('plugin', 'physics', 'gravity', 'magnetic_field', 'atmosphere', 'scene', 'light', 'spherical_coordinates'):
            if child.tag in ('physics', 'gravity', 'light', 'scene', 'magnetic_field', 'atmosphere', 'spherical_coordinates'):
                for old in world.findall(child.tag):
                    world.remove(old)
            world.append(copy.deepcopy(child))
    px4_spherical_reference(world)
    p = ET.SubElement(world, 'plugin', filename='RGLServerPluginManager', name='rgl::RGLServerPluginManager')
    ET.SubElement(p, 'do_ignore_entities_in_lidar_link').text = 'true'


def px4_spherical_reference(world):
    # PX4 MapProjection uses a 6,371,000 m sphere. Gazebo's default WGS84
    # ellipsoid otherwise gives 79.759 m east for 80 m Cartesian at this site.
    # Match the simulator's geodetic model, without calibrating against flight
    # truth, modifying state estimates, or feeding evaluator poses to control.
    coordinates = world.find('spherical_coordinates')
    if coordinates is None:
        raise ValueError('World requires an explicit geodetic origin')
    coordinates.find('surface_model').text = 'CUSTOM_SURFACE'
    for name in ('surface_axis_equatorial', 'surface_axis_polar'):
        element = coordinates.find(name)
        if element is None: element = ET.SubElement(coordinates, name)
        element.text = '6371000'


def collision_ray_surfaces(vendor):
    """RGL loads Visual geometry, including transparent visuals, not Collision.

    Retain the upstream graphics/physics, but raycast the same physical surfaces
    used for contact and clearance. These are sensor-scene surfaces, never a
    map or obstacle topic delivered to navigation.
    """
    manifest = []
    for path in sorted(vendor.rglob('model.sdf')):
        tree = ET.parse(path)
        count = 0
        for link in tree.getroot().iter('link'):
            for old in list(link.findall('visual')):
                if old.get('name', '').startswith('rgl_collision_surface_'):
                    link.remove(old)
            for i, collision in enumerate(link.findall('collision')):
                visual = ET.SubElement(link, 'visual', name=f'rgl_collision_surface_{i}')
                if collision.find('pose') is not None:
                    visual.append(copy.deepcopy(collision.find('pose')))
                visual.append(copy.deepcopy(collision.find('geometry')))
                ET.SubElement(visual, 'transparency').text = '1'
                ET.SubElement(visual, 'cast_shadows').text = 'false'
                count += 1
        if count:
            write_xml(tree.getroot(), path)
            manifest.append(dict(model=str(path.relative_to(ROOT)), surfaces=count,
                                 sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    (vendor/'RGL_COLLISION_SURFACES.json').write_text(json.dumps(dict(
        purpose='Raycast physics surfaces without changing original visual/collision geometry',
        rgl_source='https://github.com/RobotecAI/RGLGazeboPlugin/blob/d4bf3cf36fe4a363a56df1bec2ce3809720db563/RGLServerPlugin/src/Scene.cc',
        models=manifest), indent=2)+'\n')


def blocked_forest(long_root):
    root = copy.deepcopy(long_root)
    world = root.find('world'); world.set('name', 'forest_blocked')
    model = ET.SubElement(world, 'model', name='fault_barrier')
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = '8 0 2.25 0 0 0'
    link = ET.SubElement(model, 'link', name='barrier')
    for tag in ('collision', 'visual'):
        item = ET.SubElement(link, tag, name='barrier_'+tag)
        ET.SubElement(ET.SubElement(ET.SubElement(item, 'geometry'), 'box'), 'size').text = '1 40 4.5'
    write_xml(root, ROOT/'assets/gazebo_world/forest_blocked.sdf')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--px4', type=Path, default=Path.home() / 'PX4-Autopilot')
    args = parser.parse_args()
    vendor = ROOT / 'assets/vendor'
    CACHE.mkdir(parents=True, exist_ok=True)
    # ZIP contents are checked before extraction; Fuel dependencies are pinned by version.
    manifest = {}
    for archive, owner, name, version in FUEL:
        url = f'https://fuel.gazebosim.org/1.0/{owner}/models/{urllib.parse.quote(name)}/{version}/{urllib.parse.quote(name)}.zip'
        path = CACHE / archive
        if not path.exists():
            with urllib.request.urlopen(url, timeout=60) as response:
                path.write_bytes(response.read())
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest != FUEL_SHA256[archive]:
            raise ValueError('Pinned Fuel archive checksum mismatch: '+archive)
        manifest[name] = dict(owner=owner, version=version, url=url, sha256=digest,
                              license='CC-BY-4.0' if name == 'grasspatch' else 'CC0-1.0')
        dest = vendor / 'forest' / name
        dest.mkdir(parents=True, exist_ok=True)
        with zipfile.ZipFile(path) as z:
            for member in z.infolist():
                if Path(member.filename).is_absolute() or '..' in Path(member.filename).parts:
                    raise ValueError('Unsafe archive path')
            z.extractall(dest)
    (vendor / 'forest/ORIGIN.json').write_text(json.dumps(manifest, indent=2) + '\n')
    # AWS model URIs were relative to a Gazebo Classic working directory.
    for path in (vendor / 'warehouse/models').rglob('*.sdf'):
        text = path.read_text().replace('file://models/', 'model://')
        path.write_text(text)
        tree = ET.parse(path)
        # All warehouse props are static. Classic accepted invalid illustrative
        # inertia tensors on two props; omit these unused tensors in Harmonic.
        for model in tree.getroot().iter('model'):
            if model.findtext('static') in ('1', 'true'):
                for link in model.findall('link'):
                    for inertial in link.findall('inertial'):
                        link.remove(inertial)
        write_xml(tree.getroot(), path)
    source = vendor / 'warehouse/worlds/small_warehouse/small_warehouse.world'
    root = ET.parse(source).getroot()
    root.set('version', '1.9')
    world = root.find('world')
    world.set('name', 'warehouse')
    for model in world.findall('model'):
        if model.find('static') is None:
            ET.SubElement(model, 'static').text = 'true'
    for pose in world.iter('pose'):
        pose.attrib.pop('frame', None)
    systems(world)
    write_xml(root, ROOT / 'assets/gazebo_world/warehouse.sdf')
    source = ROOT/'assets/vendor/forest/forest.upstream.sdf'
    world_pin = json.loads((source.parent/'forest.world.origin.json').read_text())
    if hashlib.sha256(source.read_bytes()).hexdigest()!=world_pin['sha256']:
        raise ValueError('Pinned forest source checksum mismatch')
    root = ET.parse(source).getroot()
    world = root.find('world')
    world.set('name', 'forest_original')
    for uri in world.iter('uri'):
        if 'Pine' in uri.text:
            uri.text = 'model://Pine Tree'
        elif 'Oak' in uri.text:
            uri.text = 'model://Oak tree'
        elif 'grasspatch' in uri.text:
            uri.text = 'model://grasspatch'
    systems(world)
    write_xml(root, ROOT / 'assets/gazebo_world/forest_original.sdf')
    # Long-route generation uses the same tree meshes. The reference corridor is
    # recorded only for scoring, never loaded by the navigation entry point.
    long_root = copy.deepcopy(root)
    world = long_root.find('world')
    world.set('name', 'forest_long')
    for child in list(world):
        if child.tag in ('include', 'model'):
            world.remove(child)
    ground = ET.SubElement(world, 'model', name='ground')
    ET.SubElement(ground, 'static').text = 'true'
    link = ET.SubElement(ground, 'link', name='ground')
    for tag in ('visual', 'collision'):
        item = ET.SubElement(link, tag, name='ground_' + tag)
        geom = ET.SubElement(item, 'geometry')
        plane = ET.SubElement(geom, 'plane')
        ET.SubElement(plane, 'normal').text = '0 0 1'
        ET.SubElement(plane, 'size').text = '240 100'
    corridor = [(0, 0), (14, 0), (27, 4), (40, -4), (54, 4), (68, -3), (80, 0)]
    def center(x):
        for a, b in zip(corridor, corridor[1:]):
            if a[0] <= x <= b[0]:
                return a[1] + (b[1]-a[1]) * (x-a[0]) / (b[0]-a[0])
        return 0
    # Keep vegetation beyond B as in an extended forest, rather than ending
    # all returns exactly at the benchmark boundary. The flight remains 80 m.
    for i, x in enumerate(range(8, 113, 4)):
        for side in (-1, 1):
            # 4.5 m from pine centers to corridor; branches extend ~1.6 m.
            inc = ET.SubElement(world, 'include')
            ET.SubElement(inc, 'name').text = f'pine_{i}_{side}'
            ET.SubElement(inc, 'uri').text = 'model://Pine Tree'
            ET.SubElement(inc, 'pose').text = f'{x} {center(x)+side*4.5} 0 0 0 {i*.31}'
        inc = ET.SubElement(world, 'include')
        ET.SubElement(inc, 'name').text = f'oak_{i}'
        ET.SubElement(inc, 'uri').text = 'model://Oak tree'
        ET.SubElement(inc, 'pose').text = f'{x} {center(x)+(11 if i%2 else -11)} 0 0 0 0'
    write_xml(long_root, ROOT / 'assets/gazebo_world/forest_long.sdf')
    blocked_forest(long_root)
    score = ROOT / 'assets/evaluation'
    score.mkdir(exist_ok=True)
    (score / 'forest_reference.json').write_text(json.dumps(dict(corridor=corridor), indent=2)+'\n')
    payloads = [('avia', 'Livox Avia', .498, (.091, .061, .065), (0.12, 0, .12), .1, 60),
                ('os1_64', 'OS1 64', .425, (.085, .085, .073), (0, 0, .15), .1, 60)]
    for name, preset, mass, size, pose, near, far in payloads:
        root = ET.parse(ROOT / 'assets/gazebo_models/x500_mid360/model.sdf').getroot()
        model = root.find('model')
        model.set('name', 'x500_' + name)
        link = model.find('link')
        link.set('name', name + '_link')
        link.find('pose').text = ' '.join(map(str, (*pose, 0, 0, 0)))
        link.find('inertial/mass').text = str(mass)
        for axis, a, b in [('ixx', 1, 2), ('iyy', 0, 2), ('izz', 0, 1)]:
            link.find('inertial/inertia/' + axis).text = str(mass*(size[a]**2+size[b]**2)/12)
        link.find('collision/geometry/box/size').text = ' '.join(map(str, size))
        link.find('visual/geometry/box/size').text = ' '.join(map(str, size))
        for visual in link.findall('visual')[1:]:
            link.remove(visual)
        sensor = link.find('sensor')
        sensor.set('name', name)
        plugin = sensor.find('plugin')
        plugin.find('range/min').text = str(near)
        plugin.find('range/max').text = str(far)
        plugin.find('topic').text = '/' + name + '/points'
        plugin.find('frame').text = name + '_link'
        plugin.find('pattern_preset').text = preset
        model.find('joint').set('name', name + '_joint')
        model.find('joint/child').text = name + '_link'
        out = ROOT / 'assets/gazebo_models' / ('x500_' + name)
        write_xml(root, out / 'model.sdf')
        (out / 'model.config').write_text(f'<model><name>x500_{name}</name><version>1.0</version><sdf version="1.9">model.sdf</sdf><description>RGL interchangeable lidar payload</description></model>\n')
    collision_ray_surfaces(vendor)
    print('Prepared offline worlds and payloads. Navigation must not load assets/evaluation.')


if __name__ == '__main__':
    main()
