#!/usr/bin/env python3
"""Example: read everything from a car and (with --drive) drive it a little, from another computer.

Shows the full sensor data (every field of every sensor) and saves it: car_sensors.json (all of it),
car_lidar.csv (every lidar reading: angle, range, intensity, x, y), car_depth_points.ply (3D points from the
depth camera: open in MeshLab / CloudCompare), car_rgb.jpg, car_depth_colour.jpg, car_depth.png.

    python3 example.py http://192.168.1.50:8000                 read only: battery, sensors, pictures, video
    python3 example.py https://192.168.1.50 --insecure           the same over the local https (self-signed)
    python3 example.py http://192.168.1.50:8000 --show           also show the live video (needs opencv-python)
    python3 example.py http://192.168.1.50:8000 --drive          also drive: 20 cm forward and back (asks first)

Files are saved in the current folder.
Needs only Python 3 (mentorpi_client.py next to this file); opencv-python + numpy are optional.
"""
import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mentorpi_client import Car, CarError


def show_state(s):
    b, o, i, l = s['battery'], s['odom'] or {}, s['imu'], s['lidar'] or {}
    print(f"  mode {s['mode']}, lifted {s['safety']['lifted']}, driving {s['drive']['active']}")
    print(f"  odometry  x {o.get('x')} m, y {o.get('y')} m, heading {o.get('yaw_deg')}°, speed {o.get('speed')} m/s")
    print(f"  IMU       roll {i['roll_deg']}°, pitch {i['pitch_deg']}°, accel {i['accel_m_s2']} m/s², gyro {i['gyro_deg_s']} °/s")
    print(f"  lidar     nearest: front {l.get('front_m')} m, left {l.get('left_m')} m, back {l.get('back_m')} m, "
          f"right {l.get('right_m')} m ({l.get('readings')} readings)")
    d = s['depth']
    print(f"  depth cam nearest {d.get('nearest_camera_m')} m (only the camera sees: {d.get('nearest_camera_only_m')} m)")


def vec(v, n=3):
    """x, y, z (and w) of a ROS vector / quaternion, rounded."""
    return '(' + ', '.join(f'{v[k]:.{n}f}' if v.get(k) is not None else 'null' for k in ('x', 'y', 'z', 'w') if k in v) + ')'


def yaw_of(q):
    return math.degrees(math.atan2(2 * (q['w'] * q['z'] + q['x'] * q['y']), 1 - 2 * (q['y'] ** 2 + q['z'] ** 2)))


def show_full(full):
    """Every sensor's latest message, the main fields of each written out (car_sensors.json has every field)."""
    t = full['topics']
    print(f"  {'sensor':18s} {'topic':46s} {'rate':>8s} {'age':>7s}")
    for name, e in t.items():
        rate = f"{e['hz']} Hz" if e['hz'] else '-'
        age = f"{e['age_s']:.2f} s" if e['age_s'] is not None else 'no data'
        print(f"  {name:18s} {e['topic']:46s} {rate:>8s} {age:>7s}")
    m = lambda k: t[k]['msg']
    print()
    if m('lidar'):
        s = m('lidar')
        n = sum(r is not None for r in s['ranges'])
        print(f"  lidar          {len(s['ranges'])} readings ({n} valid), angle {s['angle_min']:.3f} .. {s['angle_max']:.3f} rad "
              f"step {s['angle_increment']:.5f}, range {s['range_min']:.2f}..{s['range_max']:.1f} m, intensities: {len(s['intensities'])}")
    for k in ('imu_raw', 'imu', 'imu_corrected'):
        if m(k):
            i = m(k)
            print(f"  {k:14s} accel {vec(i['linear_acceleration'])} m/s²  gyro {vec(i['angular_velocity'], 4)} rad/s  "
                  f"orientation {vec(i['orientation'])}")
    if m('imu_rpy'):
        v = m('imu_rpy')['vector']
        print(f"  imu_rpy        roll {math.degrees(v['x']):.2f}°  pitch {math.degrees(v['y']):.2f}°  yaw {math.degrees(v['z']):.2f}°"
              f"   standing still: {m('imu_steady')['data'] if m('imu_steady') else '?'}")
    for k in ('odom_wheels', 'odom', 'odom_lidar'):
        if m(k):
            o = m(k)
            p, tw = o['pose']['pose'], o['twist']['twist']
            print(f"  {k:14s} position {vec(p['position'])} m  heading {yaw_of(p['orientation']):.1f}°  "
                  f"speed {tw['linear']['x']:.3f} m/s  turn {tw['angular']['z']:.3f} rad/s")
    if m('battery_mv'):
        print(f"  battery_mv     {m('battery_mv')['data']} mV")
    for k in ('depth_scan', 'depth_bumps', 'depth_drops'):
        if m(k):
            r = [x for x in m(k)['ranges'] if x is not None]
            print(f"  {k:14s} {len(r)} directions with something, nearest {min(r):.2f} m" if r else f"  {k:14s} nothing")
    for k in ('rgb_camera_info', 'depth_camera_info'):
        if m(k):
            c = m(k)
            print(f"  {k:18s} {c['width']}x{c['height']}  fx {c['k'][0]:.1f} fy {c['k'][4]:.1f} cx {c['k'][2]:.1f} "
                  f"cy {c['k'][5]:.1f}  distortion {c['distortion_model']}")
    if m('command'):
        c = m('command')
        print(f"  command        speed {c['linear']['x']:.2f} m/s, steering {math.degrees(c['angular']['z']):.1f}°")
    if m('button'):
        print(f"  button         id {m('button')['id']} state {m('button')['state']}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('url', help='the car page, e.g. http://192.168.1.50:8000')
    ap.add_argument('--insecure', action='store_true', help="accept the car's self-signed https certificate")
    ap.add_argument('--show', action='store_true', help='show the live video in a window (opencv-python)')
    ap.add_argument('--drive', action='store_true', help='drive about 20 cm forward and back (asks first)')
    a = ap.parse_args()
    car = Car(a.url, verify=not a.insecure)

    # 1. Who is it?
    try:
        info = car.info()
    except CarError as e:
        sys.exit(f'No car there: {e}')
    print(f"Car {info['car']} (API v{info['api']}), limits: {info['limits']}")

    # 2. Battery and charging
    b = car.battery()
    eta = f", {b['eta_min']:.0f} min {b['eta_kind']}" if b.get('eta_min') else ''
    print(f"\nBattery: {b['voltage']} V ({b['level']}), about {b.get('soc')} %, {b.get('state')}{eta}; "
          f"charging mode {b['charging_mode']}")

    # 3. All sensors at once
    print('\nState:')
    show_state(car.state())

    # 4. FULL sensor data: every sensor's latest message, every field
    print('\nFull sensor data (every topic; all fields in car_sensors.json):')
    full = car.sensors()
    show_full(full)
    with open('car_sensors.json', 'w') as f:
        json.dump(full, f, indent=1)
    s = full['topics']['lidar']['msg']
    if s:
        lx = full['frames']['lidar']['x']
        with open('car_lidar.csv', 'w') as f:
            f.write('angle_rad,range_m,intensity,x_m,y_m\n')
            for i, r in enumerate(s['ranges']):
                ang = s['angle_min'] + i * s['angle_increment']
                inten = s['intensities'][i] if i < len(s['intensities']) else ''
                xy = f'{lx + r * math.cos(ang):.3f},{r * math.sin(ang):.3f}' if r is not None else ','
                f.write(f"{ang:.5f},{'' if r is None else round(r, 4)},{inten},{xy}\n")
    open('car_depth_points.ply', 'wb').write(car.depth_points(step=4, ply=True))
    print('\nSaved car_sensors.json (everything), car_lidar.csv (every lidar reading), '
          'car_depth_points.ply (3D points, car frame)')

    # 5. The whole lidar scan, as points around the car
    pts = car.lidar_points()
    ahead = [x for x, y in pts if x > 0 and abs(y) < 0.1]
    print(f'\nLidar: {len(pts)} points; nearest straight ahead (car width): {min(ahead):.2f} m' if ahead else
          f'\nLidar: {len(pts)} points; nothing straight ahead')

    # 6. Pictures
    open('car_rgb.jpg', 'wb').write(car.picture())
    open('car_depth_colour.jpg', 'wb').write(car.picture('depth'))
    png = car.depth_png()
    open('car_depth.png', 'wb').write(png)
    print('\nSaved car_rgb.jpg, car_depth_colour.jpg and car_depth.png (16-bit, millimetres)')
    try:
        import cv2
        import numpy as np
        depth = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_UNCHANGED)
        h, w = depth.shape
        centre = depth[h // 2 - 10:h // 2 + 10, w // 2 - 10:w // 2 + 10]
        valid = centre[centre > 0]
        print(f'  depth image {w}x{h}; centre of the picture: '
              + (f'{np.median(valid) / 1000:.2f} m away' if valid.size else 'no reading'))
    except ImportError:
        cv2 = None
        print('  (pip install opencv-python numpy to read the depth image as numbers)')

    # 7. Live state for 3 s
    print('\nLive state, 5 times a second for 3 s:')
    t0 = time.time()
    for s in car.stream(hz=5):
        o = s['odom'] or {}
        print(f"  {time.time() - t0:4.1f} s  speed {o.get('speed')} m/s  heading {o.get('yaw_deg')}°  "
              f"front {(s['lidar'] or {}).get('front_m')} m  battery {s['battery']['voltage']} V")
        if time.time() - t0 > 3:
            break

    # 8. Live video for 3 s (or until q in the window)
    n, t0 = 0, time.time()
    for jpeg in car.video(width=640):
        n += 1
        if a.show and cv2 is not None:
            frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            cv2.imshow(f"{info['car']} camera (q: close)", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        elif time.time() - t0 > 3:
            break
    print(f'\nVideo: {n} frames in {time.time() - t0:.1f} s ({n / (time.time() - t0):.1f} per second)')

    # 9. Driving
    print('\nDriving check (dry run, the car does not move):')
    for speed, steer, what in ((0.15, 0, 'forward'), (-0.15, 0, 'backward'), (0.15, 30, 'forward, steering left')):
        r = car.drive(speed, steer, duration=1.0, dry=True)
        room = r.get('path_clear_m')
        print(f"  {what:24s} -> {'ok' if r['ok'] else 'NO'}: {r['message']}"
              f"  (free path: {'nothing in the way' if room is None else f'{room} m'})")
    if a.drive:
        if input('\nThe car will drive about 20 cm forward, then back. Is the way clear and someone watching? '
                 'Type yes: ').strip().lower() != 'yes':
            print('Not driving.')
            return
        try:
            r = car.drive_for(0.15, steer=0, seconds=1.5)
            print('  forward:', r.get('message'))
            time.sleep(0.5)
            r = car.drive_for(-0.15, steer=0, seconds=1.5)
            print('  back:   ', r.get('message'))
            car.horn()
        finally:
            car.stop()
        print('  moved, now at', car.odom())


if __name__ == '__main__':
    main()
