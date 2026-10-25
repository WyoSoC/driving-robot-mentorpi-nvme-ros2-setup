"""Replanning around a new obstacle (tests/run.sh test_replan.py [bt.xml]), the real Nav2 stack without the car:
nav/nav.launch.py slam:=true on its own ROS domain (77: the car's drivers don't see it), a made-up 6 x 4 m
map as /team/map, the car standing still at (0.5, 2.0) (static TF). A goal at (5.5, 2.0) is sent; 4 s later
a wall is put across the straight path (new /team/map). Measured: how long until Nav2 publishes a plan that
goes around it. With the path validity check (navigate_ackermann.xml) it must come within ~2 s; with
replanning only on a timer it waits for the timer. Doesn't drive anything."""
import math, os, subprocess, sys, time
DOMAIN = '77'
os.environ['ROS_DOMAIN_ID'] = DOMAIN
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster

BT = sys.argv[1] if len(sys.argv) > 1 else '/home/ubuntu/nav/navigate_ackermann.xml'
RES, W, H = 0.05, 120, 80                    # 6 x 4 m
WALL = (2.6, 2.8, 1.2, 2.8)                  # x0, x1, y0, y1 of the wall put across the path


def grid(wall):
    g = OccupancyGrid()
    g.header.frame_id = 'map'
    g.info.resolution, g.info.width, g.info.height = RES, W, H
    g.info.origin.orientation.w = 1.0
    d = [0] * (W * H)
    for r in range(H):
        for c in range(W):
            if r in (0, H - 1) or c in (0, W - 1):
                d[r * W + c] = 100
            x, y = (c + 0.5) * RES, (r + 0.5) * RES
            if wall and WALL[0] <= x <= WALL[1] and WALL[2] <= y <= WALL[3]:
                d[r * W + c] = 100
    g.data = d
    return g


def blocked(path):
    return any(WALL[0] - 0.1 <= p.pose.position.x <= WALL[1] + 0.1 and WALL[2] - 0.1 <= p.pose.position.y <= WALL[3] + 0.1
               for p in path.poses)


def main():
    params = '/tmp/nav2_params_test.yaml'
    import yaml
    P = yaml.safe_load(open('/home/ubuntu/nav/nav2_params.yaml'))
    P['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = BT
    yaml.safe_dump(P, open(params, 'w'))
    nav = subprocess.Popen(['ros2', 'launch', '/home/ubuntu/nav/nav.launch.py', 'slam:=true', f'params:={params}'],
                           env=dict(os.environ, ROS_DOMAIN_ID=DOMAIN), start_new_session=True,
                           stdout=open('/tmp/replan_nav.log', 'w'), stderr=subprocess.STDOUT)
    rclpy.init()
    n = rclpy.create_node('replan_test')
    try:
        tfb = StaticTransformBroadcaster(n)
        ts = []
        for parent, child, x, y in (('map', 'odom', 0.0, 0.0), ('odom', 'base_footprint', 0.5, 2.0)):
            t = TransformStamped(); t.header.frame_id, t.child_frame_id = parent, child
            t.transform.translation.x, t.transform.translation.y = x, y; t.transform.rotation.w = 1.0
            ts.append(t)
        tfb.sendTransform(ts)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        mp = n.create_publisher(OccupancyGrid, '/team/map', qos)
        mp.publish(grid(False))
        plans = []
        n.create_subscription(Path, '/plan', lambda m: plans.append((time.monotonic(), m)), 10)
        cli = n.create_client(Trigger, '/lifecycle_manager_navigation/is_active')
        t0, active = time.monotonic(), False
        while time.monotonic() - t0 < 90 and not active:
            rclpy.spin_once(n, timeout_sec=1.0)
            if cli.service_is_ready():
                f = cli.call_async(Trigger.Request()); rclpy.spin_until_future_complete(n, f, timeout_sec=2)
                active = bool(f.result() and f.result().success)
        print(f'Nav2 active: {active} ({time.monotonic() - t0:.0f} s); behaviour tree {os.path.basename(BT)}')
        assert active, 'Nav2 did not come up (see /tmp/replan_nav.log)'
        ac = ActionClient(n, NavigateToPose, 'navigate_to_pose'); ac.wait_for_server(timeout_sec=10)
        g = NavigateToPose.Goal(); g.pose.header.frame_id = 'map'
        g.pose.pose.position.x, g.pose.pose.position.y, g.pose.pose.orientation.w = 5.5, 2.0, 1.0
        fut = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, fut, timeout_sec=5)
        assert fut.result() and fut.result().accepted, 'goal not accepted (behaviour tree did not load?)'
        end = time.monotonic() + 4.0
        while time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=0.1)
        assert plans, 'no plan before the wall'
        print(f'1. first plan: {len(plans[-1][1].poses)} poses, straight through where the wall will be: {blocked(plans[-1][1])}')
        wall_at = time.monotonic()
        mp.publish(grid(True))
        around = None
        while time.monotonic() - wall_at < 12 and around is None:
            rclpy.spin_once(n, timeout_sec=0.1)
            for t, m in plans:
                if t > wall_at and m.poses and not blocked(m):
                    around = t - wall_at
                    break
        print(f'2. wall across the path: a plan around it after {"%.1f s" % around if around is not None else "more than 12 s"}')
        print('RESULT', around)
        ac_goal = fut.result(); ac_goal.cancel_goal_async()
        for _ in range(10):
            rclpy.spin_once(n, timeout_sec=0.1)
        return around
    finally:
        os.killpg(nav.pid, 2)
        try:
            nav.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(nav.pid, 9)
        n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    around = main()
    if len(sys.argv) == 1:
        assert around is not None and around < 2.5, 'too slow to go around a new obstacle'
        print('ALL OK')
