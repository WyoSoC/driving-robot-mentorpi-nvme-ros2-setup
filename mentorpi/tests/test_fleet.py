"""Fleet test without driving: real Nav2 costmap node + fleet.py with a fake other car.
Run: tests/run.sh test_fleet.py (on a car; starts a costmap node of its own, nothing drives)."""
import math, subprocess, sys, time, yaml
sys.path.insert(0, '/home/ubuntu/web_ui')
import numpy as np, rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
import fleet

# ---- 1. pure logic: right of way ---------------------------------------------
line = [[x / 20, 0.0] for x in range(0, 60)]          # path along +x, 3 m
assert fleet.blocks(line, (0, 0), (1.0, 0.1))
assert not fleet.blocks(line, (0, 0), (1.0, 0.5))
assert not fleet.blocks(line, (0, 0), (2.0, 0.0))       # beyond 1.2 m ahead
assert fleet.blocks(line, (1.5, 0), (2.5, 0.0))         # ahead measured from where the car is
assert fleet.right_of_way('jpi1', 'jpi2') and not fleet.right_of_way('jpi2', 'jpi1')
print('logic ok')

class FakeNav:
    def __init__(s): s.pose=[0.0,0.0,0.0]; s.plan=line; s.goal_state='active'; s.goal=[3,0,0]; s.paused=0; s.resumed=0
    def running(s): return True
    def goal_active(s): return s.goal_state in ('sending','active')
    def status(s): return {'running':True,'slam_mode':False,'map':{'hash':'abc'},'pose_set':True,'pose':s.pose}
    def pause(s, r): s.goal_state='yielding'; s.paused+=1
    def resume(s): s.goal_state='active'; s.resumed+=1; return True,''
class FakePeers:
    def __init__(s): s.st=None
    def summary(s):
        return [{'car':'jpi2' if s.me=='jpi1' else 'jpi1','online':True,'age_s':0.3,'status':s.st}]
def peer(x,y,yaw,state='active',plan=()):
    return {'nav':{'running':True,'map_hash':'abc','pose_set':True,'pose':[x,y,yaw],'goal_state':state,'plan':list(plan)}}

# ---- 2. costmap with the real layer config ------------------------------------
P = yaml.safe_load(open('/home/ubuntu/nav/nav2_params.yaml'))['global_costmap']['global_costmap']['ros__parameters']
cm = {k: P[k] for k in ('footprint', 'footprint_padding', 'resolution', 'fleet_layer')}
cm.update(global_frame='map', robot_base_frame='base_footprint', rolling_window=False, width=8, height=8,
          origin_x=-4.0, origin_y=-4.0, update_frequency=5.0, publish_frequency=5.0, track_unknown_space=False,
          plugins=['fleet_layer'], always_send_full_costmap=True, transform_tolerance=1.0)
yaml.safe_dump({'costmap': {'costmap': {'ros__parameters': cm}}}, open('/tmp/fleet_cm.yaml', 'w'))
procs = [subprocess.Popen(['ros2','run','tf2_ros','static_transform_publisher','--frame-id','map','--child-frame-id','base_footprint'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL),
         subprocess.Popen(['ros2','run','nav2_costmap_2d','nav2_costmap_2d','--ros-args','-r','__ns:=/costmap','--params-file','/tmp/fleet_cm.yaml'],stdout=open('/tmp/fleet_cm.log','w'),stderr=subprocess.STDOUT)]
try:
    time.sleep(4)
    for t in ('configure','activate'):
        print(t, subprocess.run(['ros2','lifecycle','set','/costmap/costmap',t],capture_output=True,text=True).stdout.strip())
    rclpy.init()
    node = Node('fleet_test')
    tf = Buffer(); TransformListener(tf, node)
    grid = {}
    node.create_subscription(OccupancyGrid, '/costmap/costmap', lambda m: grid.__setitem__('g', m), 1)
    nav, peers = FakeNav(), FakePeers(); peers.me='jpi1'
    f = fleet.Fleet(node, tf, nav, peers, 'jpi1')
    def spin(s):
        end=time.time()+s
        while time.time()<end: rclpy.spin_once(node, timeout_sec=0.05)
    def lethal_near(x, y, r=0.12):
        g=grid['g']; a=np.array(g.data).reshape(g.info.height,g.info.width); res=g.info.resolution
        ox,oy=g.info.origin.position.x,g.info.origin.position.y
        i0,i1=int((y-r-oy)/res),int((y+r-oy)/res); j0,j1=int((x-r-ox)/res),int((x+r-ox)/res)
        return int((a[i0:i1+1,j0:j1+1]>=99).sum())
    for _ in range(100):
        spin(0.1)
        if 'g' in grid: break
    # other car parked beside the path at (1.5, 1.0): marked, no yielding
    peers.st = peer(1.5, 1.0, 0.0, state='succeeded'); spin(2.0)
    print('parked car marked cells:', lethal_near(1.5,1.0), ' yielding:', nav.goal_state=='yielding')
    assert lethal_near(1.5,1.0) > 10 and nav.goal_state=='active'
    # it drives on to (-2, -2): old marks gone
    peers.st = peer(-2.0, -2.0, 0.0, state='active'); spin(2.0)
    print('after moving: old place', lethal_near(1.5,1.0), ' new place', lethal_near(-2,-2))
    g=grid['g']; a=np.array(g.data).reshape(g.info.height,g.info.width)
    for i,j in zip(*np.nonzero(a>=99)):
        x,y=g.info.origin.position.x+(j+.5)*0.05, g.info.origin.position.y+(i+.5)*0.05
        if x>0: print('  left at', round(x,3), round(y,3), a[i,j])
    spin(3.0); print('  3 s later old place', lethal_near(1.5,1.0))
    assert lethal_near(1.5,1.0) == 0 and lethal_near(-2,-2) > 10
    # it drives across this car's path at (0.8, 0.1): pause
    peers.st = peer(0.8, 0.1, math.pi/2, plan=[[0.8, 0.1+i/20] for i in range(30)]); spin(1.0)
    print('crossing car -> this car', nav.goal_state); assert nav.goal_state=='yielding' and f.yield_to=='jpi2'
    # it has passed (0.8, 1.2): resume after CLEAR_S
    peers.st = peer(0.8, 1.2, math.pi/2); spin(1.0); assert nav.goal_state=='yielding'
    spin(2.0); print('passed ->', nav.goal_state, '|', f.note); assert nav.goal_state=='active'
    # head-on, both block each other: jpi1 keeps going, jpi2 waits
    peers.st = peer(0.9, 0.0, math.pi, plan=[[0.9-i/20, 0.0] for i in range(40)]); spin(1.0)
    print('head-on as jpi1 ->', nav.goal_state); assert nav.goal_state=='active'
    f.me='jpi2'; spin(1.0); print('head-on as jpi2 ->', nav.goal_state); assert nav.goal_state=='yielding'
    # offline other car: nothing marked
    peers.st = None; spin(2.0); print('other car offline: marked', lethal_near(0.9,0.0)); assert lethal_near(0.9,0.0)==0
    print('ALL OK')
finally:
    for p in procs: p.terminate()
    subprocess.run(['pkill','-f','[n]av2_costmap_2d --ros-args'])
