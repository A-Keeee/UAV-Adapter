#!/usr/bin/env python3
"""Offline SDF/mesh check of the unarmed scene pose; never starts Gazebo/PX4."""
import itertools,json,math,os
from pathlib import Path
import subprocess,sys,xml.etree.ElementTree as ET
import numpy as np
import trimesh
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
from autonomy_scene import spawn_pose
from autonomy_geometry import mesh_file,pose,resolve


def main():
    floor_model=next(m for m in ET.parse(ROOT/'assets/gazebo_world/warehouse.sdf').getroot().find('world').findall('model') if m.attrib['name']=='aws_robomaker_warehouse_GroundB_01_001')
    floor_sdf=ET.parse(resolve(floor_model.findtext('include/uri'))/'model.sdf').getroot().find('model')
    floor=mesh_file(str(resolve(floor_sdf.findtext('link/collision/geometry/mesh/uri')))).copy()
    floor.apply_transform(pose(floor_model.find('pose'))@pose(floor_sdf.find('pose')))
    floor_top=float(floor.bounds[1,2]);assert .034<floor_top<.035
    env=dict(os.environ)
    px4=Path.home()/'PX4-Autopilot'
    paths=[ROOT/'assets/gazebo_models',ROOT/'assets/vendor/warehouse/models',ROOT/'assets/vendor/forest',px4/'Tools/simulation/gz/models']
    env['GZ_SIM_RESOURCE_PATH']=os.pathsep.join(map(str,paths))+os.pathsep+env.get('GZ_SIM_RESOURCE_PATH','');env['SDF_PATH']=env['GZ_SIM_RESOURCE_PATH']
    results=[]
    for lidar in ('mid360','avia','os1_64'):
        xml=subprocess.run(['gz','sdf','-p',str(ROOT/'assets/gazebo_models'/f'x500_{lidar}'/'model.sdf')],env=env,capture_output=True,text=True,check=True).stdout
        model=ET.fromstring(xml).find('model');frames={n.attrib['name']:n for k in ('frame','link') for n in model.findall(k)};transforms={'__model__':np.eye(4)}
        def world(name):
            if name not in transforms:
                n=frames[name];p=n.find('pose');parent=p.attrib.get('relative_to','__model__') if p is not None else '__model__';transforms[name]=world(parent)@pose(p)
            return transforms[name]
        corners=[]
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                g=collision.find('geometry')
                if g.find('box') is not None:extent=np.array(list(map(float,g.findtext('box/size').split())))/2
                elif g.find('cylinder') is not None:extent=np.array([float(g.findtext('cylinder/radius'))]*2+[float(g.findtext('cylinder/length'))/2])
                elif g.find('sphere') is not None:extent=np.repeat(float(g.findtext('sphere/radius')),3)
                else:raise AssertionError('Unsupported collision, do not skip it')
                t=world(link.attrib['name'])@pose(collision.find('pose'))
                corners.extend((t@np.r_[extent*np.array(signs),1])[:3] for signs in itertools.product((-1,1),repeat=3))
        for yaw in (0.,math.pi/4,-math.pi/4):
            initial=spawn_pose('warehouse',yaw);t=trimesh.transformations.euler_matrix(*initial[3:]);t[:3,3]=initial[:3];points=trimesh.transform_points(np.array(corners),t)
            gap=float(points[:,2].min()-floor_top)
            assert gap>.05,'Warehouse unarmed spawn intersects floor or lacks settling margin'
            old_gap=gap-initial[2];assert old_gap<-.02,'Recorded zero-pose penetration witness no longer reproduced'
            results.append(dict(lidar=lidar,yaw=yaw,old_floor_gap_m=old_gap,new_floor_gap_m=gap,model_spawn=list(initial)))
    assert spawn_pose('forest_long',.7)==(0.,0.,0.,0.,0.,.7),'Forest setup pose changed'
    print(json.dumps(dict(floor_top_m=floor_top,configurations=results)))
    print('All nine payload/yaw combinations spawn above the warehouse floor; forest pose unchanged: PASS')

if __name__=='__main__':main()
