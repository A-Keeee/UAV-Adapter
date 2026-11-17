#!/usr/bin/env python3
"""Check the navigation body envelope against every expanded SDF collision box.

Use conservative bounding boxes for cylinders/spheres. This tool reads geometry
only for offline verification; it does not feed Gazebo truth into navigation.
"""
import hashlib
import itertools
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
ROOT=Path(__file__).resolve().parents[1]

def check(path):
    model=ET.parse(path).getroot().find('model')
    frames={node.attrib['name']:node for kind in ('frame','link') for node in model.findall(kind)}
    poses={'__model__':np.eye(4)}
    def local(node):
        pose=node.find('pose'); values=[float(v) for v in pose.text.split()] if pose is not None and pose.text else [0.]*6
        matrix=trimesh.transformations.euler_matrix(*values[3:]);matrix[:3,3]=values[:3]
        return (pose.attrib.get('relative_to','__model__') if pose is not None else '__model__'),matrix
    def world(name):
        if name not in poses:
            parent,matrix=local(frames[name]);poses[name]=world(parent)@matrix
        return poses[name]
    rotating={joint.findtext('child') for joint in model.findall('joint') if joint.attrib.get('type')=='revolute'}
    origin=np.linalg.inv(world('base_link')); corners=[]; counts=0
    for link in model.findall('link'):
        for collision in link.findall('collision'):
            _,matrix=local(collision); transform=origin@world(link.attrib['name'])@matrix
            geom=collision.find('geometry')
            if geom.find('box') is not None: extent=np.array([float(v) for v in geom.findtext('box/size').split()])/2
            elif geom.find('cylinder') is not None:
                radius=float(geom.findtext('cylinder/radius'));extent=np.array([radius,radius,float(geom.findtext('cylinder/length'))/2])
            elif geom.find('sphere') is not None: extent=np.repeat(float(geom.findtext('sphere/radius')),3)
            else: raise ValueError('Unsupported collision '+collision.attrib['name'])
            if link.attrib['name'] in rotating:
                # Bound the complete rotor sweep, not a single propeller angle.
                radius=float(np.linalg.norm(extent[:2]));extent[:2]=radius
            for signs in itertools.product((-1,1),repeat=3):corners.append((transform@np.r_[extent*np.array(signs),1])[:3])
            counts+=1
    corners=np.array(corners);half=np.array([.50,.50,.35]); norm=np.linalg.norm(corners/half,axis=1)
    if norm.max()>.905:raise ValueError('Airframe exceeds the verified 0.905 hold-anchor enclosure bound')
    return dict(model=model.attrib['name'],collisions=counts,geometry_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        maximum_normalized_radius=float(norm.max()),bounding_half_extent_m=np.abs(corners).max(axis=0).tolist())

def main():
    results=[]
    for lidar in ('mid360','avia','os1_64'):
        matches=sorted((ROOT/'log/autonomy_scene').glob(f'*/models/x500_{lidar}/model.sdf'))
        if not matches:raise ValueError('Run the unarmed scene check for '+lidar+' first')
        results.append(check(matches[-1]))
    out=ROOT/'assets/evaluation/airframe_envelope.json'
    out.write_text(json.dumps(dict(observed_swept_ellipsoid_half_axes_m=[.5,.5,.35],results=results),indent=2)+'\n')
    print(out);print('Three payload airframes fit the observed swept-body envelope: PASS')
if __name__=='__main__':main()
