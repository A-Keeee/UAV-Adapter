"""Native Swarm in isolation, exact spline tracking fixture, no vehicle or Gazebo."""
import os,sys,json,time,subprocess,signal,tempfile
from pathlib import Path
os.environ['ROS_DOMAIN_ID']='77';os.environ['ROS_LOCALHOST_ONLY']='1'
import numpy as np
from scipy.interpolate import BSpline
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header,Float64
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from uav_common_msg.msg import NativeTrajectory,TrajectoryFeedback
root=Path(__file__).resolve().parents[3];speed=float(sys.argv[1]) if len(sys.argv)>1 else .18
assert speed in (.18,.5)
budget=90 if speed==.18 else 45
rclpy.init();n=Node('native_progress_probe')
clock=n.create_publisher(Clock,'/clock',qos_profile_sensor_data);odom=n.create_publisher(Odometry,'/probe/odom',10);cloud=n.create_publisher(PointCloud2,'/probe/cloud',qos_profile_sensor_data);goal=n.create_publisher(PoseStamped,'/autonomy/goal',10);ack=n.create_publisher(TrajectoryFeedback,'/autonomy/trajectory_feedback',10);limit=n.create_publisher(Float64,'/autonomy/speed_limit',10)
ns=1_000_000_000;current=1.;pos=np.array([-1.7,1.5,5.1]);target=np.array([.0084,.0274,3.77485]);curve=None;start=0.;count=0;rejected=0;records=[];maxima=np.zeros(3)
def receive(m):
 global curve,start,count,rejected,maxima
 f=BSpline(np.array(m.knots),np.array(m.control_points).reshape(-1,3),m.degree);duration=m.knots[-m.degree-1]-m.knots[m.degree]
 bound=np.array([np.linalg.norm(f.derivative(d)(np.linspace(0,duration,300)),axis=1).max() for d in (1,2,3)])
 heights=f(np.linspace(0,duration,300))[:,2]
 valid=duration<=300 and np.all(bound<np.array([speed+1e-6,.700001,1.500001])) and heights.min()>=3.7 and heights.max()<=5.7
 if valid:curve=f;start=m.start_time.sec+m.start_time.nanosec*1e-9;count+=1;maxima=np.maximum(maxima,bound)
 else:rejected+=1
 ack.publish(TrajectoryFeedback(trajectory_id=m.trajectory_id,accepted=bool(valid),braking=False))
n.create_subscription(NativeTrajectory,'/autonomy/native_trajectory',receive,10)
overlay=Path.home()/'.cache/gazebo_ego_swarm/install'
args=[str(overlay/'ego_planner/lib/ego_planner/ego_planner_node'),'--ros-args','--params-file',str(root/'uav_autonomy/config/ego_swarm.yaml'),'-r','odom_world:=/probe/odom','-r','grid_map/odom:=/probe/odom','-r','grid_map/cloud:=/probe/cloud','-r','planning/bspline:=/ego/native_trajectory','-r','/move_base_simple/goal:=/ego/goal']
commands=[args,[str(overlay/'ego_ganyu_bridge/lib/ego_ganyu_bridge/ego_ganyu_bridge'),'--ros-args','-p','use_sim_time:=true']];ps=[];sent=False;arrived=None
with tempfile.TemporaryFile() as logs:
 try:
  for cmd in commands:ps.append(subprocess.Popen(cmd,stdout=logs,stderr=subprocess.STDOUT,start_new_session=True))
  time.sleep(.5)
  while current<budget+4:
   ns+=20_000_000;current=ns/1e9;c=Clock();c.clock.sec,c.clock.nanosec=divmod(ns,10**9);clock.publish(c)
   vel=np.zeros(3)
   if curve is not None:
    t=min(current-start,curve.t[-4]);pos=curve(t);vel=curve.derivative(1)(t)
   o=Odometry();o.header=Header(stamp=c.clock,frame_id='odom');o.pose.pose.orientation.w=1.;o.pose.pose.position.x,o.pose.pose.position.y,o.pose.pose.position.z=map(float,pos);o.twist.twist.linear.x,o.twist.twist.linear.y,o.twist.twist.linear.z=map(float,vel);odom.publish(o)
   if ns%100_000_000==0:
    cloud.publish(create_cloud_xyz32(o.header,[[8.,8.,4.],[8.,-8.,4.],[-8.,8.,4.],[-8.,-8.,4.]]));limit.publish(Float64(data=.8*speed/np.sqrt(3.)))
   if current>3 and not sent:
    g=PoseStamped();g.header=o.header;g.pose.orientation.w=1.;g.pose.position.x,g.pose.position.y,g.pose.position.z=map(float,target);goal.publish(g);sent=True
   for _ in range(4):rclpy.spin_once(n,timeout_sec=0)
   if ns%1_000_000_000==0:records.append(dict(t=current,p=pos.tolist(),distance=float(np.linalg.norm(pos-target))))
   if sent and np.linalg.norm(pos-target)<.15 and np.linalg.norm(vel)<.1:arrived=current-3;break
   time.sleep(.005)
  result=dict(execution_speed_limit=speed,arrival_sim_seconds=arrived,accepted=count,rejected=rejected,max_vaj=maxima.tolist(),final_distance=float(np.linalg.norm(pos-target)),physical_flight=False)
  print(json.dumps(result),flush=True)
  assert arrived is not None and arrived<=budget, 'Native guarded-speed return did not converge within its regression budget'
  assert count>3 and rejected==0, 'Fixture did not exercise repeated feasible native replanning'
  print('Native Swarm low-speed diagonal return converges with original spline timing: PASS')
 except BaseException:
  logs.seek(0);print(logs.read().decode(),file=sys.stderr);raise
 finally:
  for p in ps:
   if p.poll() is None:os.killpg(p.pid,signal.SIGTERM)
  for p in ps:
   try:p.wait(timeout=3)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
  n.destroy_node();rclpy.shutdown()
