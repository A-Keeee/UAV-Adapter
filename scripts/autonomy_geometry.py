"""Scoring-only collision mesh loader. Never imported by navigation nodes."""
from functools import lru_cache
from pathlib import Path
import xml.etree.ElementTree as ET
import collada
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
SEARCH = [ROOT/'assets/vendor/warehouse/models', ROOT/'assets/vendor/forest']


def pose(element):
    values = list(map(float, (element.text or '0 0 0 0 0 0').split())) if element is not None else [0]*6
    result = trimesh.transformations.euler_matrix(*values[3:6])
    result[:3, 3] = values[:3]
    return result


def resolve(uri):
    if uri.startswith('model://'):
        rel = uri[len('model://'):]
        for base in SEARCH:
            candidate = base/rel
            if candidate.exists(): return candidate
        raise FileNotFoundError(uri)
    if uri.startswith('file://'): uri = uri[7:]
    p = Path(uri)
    if p.is_absolute() and p.exists(): return p
    raise FileNotFoundError(uri)


@lru_cache(maxsize=100)
def mesh_file(path):
    p = Path(path)
    if p.suffix.lower() == '.dae':
        dae = collada.Collada(str(p))
        meshes = []
        for geometry in dae.scene.objects('geometry'):
            for primitive in geometry.primitives():
                if hasattr(primitive, 'triangleset'): primitive = primitive.triangleset()
                if not hasattr(primitive, 'vertex_index'): continue
                meshes.append(trimesh.Trimesh(vertices=np.asarray(primitive.vertex)*float(dae.assetInfo.unitmeter or 1),
                                               faces=primitive.vertex_index, process=False))
        return trimesh.util.concatenate(meshes)
    return trimesh.load(str(p), force='mesh')


def world_collision_mesh(world_path):
    meshes = []
    planes = []
    def include(element, parent):
        folder = resolve(element.findtext('uri'))
        model(ET.parse(folder/'model.sdf').getroot().find('model'), parent@pose(element.find('pose')))
    def model(element, parent):
        transform = parent@pose(element.find('pose'))
        for child in element.findall('include'): include(child, transform)
        for child in element.findall('model'): model(child, transform)
        for link in element.findall('link'):
            for collision in link.findall('collision'):
                matrix = transform@pose(link.find('pose'))@pose(collision.find('pose'))
                geom = collision.find('geometry')
                if geom.find('plane') is not None:
                    planes.append(float(matrix[2, 3])); continue
                if geom.find('mesh') is not None:
                    item = geom.find('mesh'); mesh = mesh_file(str(resolve(item.findtext('uri')))).copy()
                    scale = list(map(float, item.findtext('scale', '1 1 1').split()))
                    mesh.apply_scale(scale)
                elif geom.find('box') is not None:
                    mesh = trimesh.creation.box(extents=list(map(float, geom.findtext('box/size').split())))
                elif geom.find('cylinder') is not None:
                    mesh = trimesh.creation.cylinder(radius=float(geom.findtext('cylinder/radius')), height=float(geom.findtext('cylinder/length')), sections=32)
                elif geom.find('sphere') is not None:
                    mesh = trimesh.creation.icosphere(radius=float(geom.findtext('sphere/radius')), subdivisions=2)
                else: raise ValueError('Unsupported scoring geometry: '+ET.tostring(geom).decode())
                mesh.apply_transform(matrix); meshes.append(mesh)
    world = ET.parse(world_path).getroot().find('world')
    for element in world.findall('model'): model(element, np.eye(4))
    for element in world.findall('include'): include(element, np.eye(4))
    mesh = trimesh.util.concatenate(meshes)
    return mesh, planes
