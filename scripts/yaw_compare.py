#!/usr/bin/env python3
"""Compare IMU and odom yaw at matching header timestamps. ROS 2 required."""
import argparse
import bisect
import csv
import math
import time
from collections import deque
from pathlib import Path


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


def yaw(q):
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('nonfinite quaternion')
    norm = math.sqrt(sum(v*v for v in values))
    if norm < 1e-8:
        raise ValueError('zero quaternion')
    x, y, z, w = (v/norm for v in values)
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9


def interpolate(samples, t, max_gap=0.04):
    i = bisect.bisect_left([s[0] for s in samples], t)
    if i < len(samples) and abs(samples[i][0]-t) < 1e-7:
        return samples[i][1:]
    if i == 0 or i == len(samples):
        return None
    a, b = samples[i-1], samples[i]
    if b[0]-a[0] > max_gap or b[0] <= a[0]:
        return None
    f = (t-a[0])/(b[0]-a[0])
    return (wrap(a[1]+f*wrap(b[1]-a[1])), a[2]+f*(b[2]-a[2]))


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--odom', default='/localization/kinematic_state')
    p.add_argument('--imu', default='/sensing/imu/imu_data')
    args = p.parse_args()
    class Compare(Node):
        def __init__(self):
            super().__init__('yaw_compare')
            self.samples = deque(maxlen=3000)
            self.pending = deque(maxlen=100)
            self.last = None
            self.total = [0., 0.]
            self.last_print = 0.
            self.file = args.output.open('x', newline='')
            self.csv = csv.writer(self.file)
            self.csv.writerow(['stamp','imu_yaw_rad','odom_yaw_rad','gyro_z_rad_s',
                               'imu_turn_deg','odom_turn_deg','odom_minus_imu_deg'])
            self.create_subscription(Imu,args.imu,self.imu,qos_profile_sensor_data)
            self.create_subscription(Odometry,args.odom,self.odom,qos_profile_sensor_data)
            self.create_timer(.05,self.flush)
        def imu(self,msg):
            if msg.orientation_covariance[0] < 0 or not math.isfinite(msg.angular_velocity.z): return
            try: value = yaw(msg.orientation)
            except ValueError: return
            t = stamp(msg)
            if self.samples and t <= self.samples[-1][0]: return
            self.samples.append((t,value,msg.angular_velocity.z))
        def odom(self,msg):
            try: value = yaw(msg.pose.pose.orientation)
            except ValueError: return
            self.pending.append((stamp(msg),value,time.monotonic()))
        def flush(self):
            while self.pending:
                t,o,arrival = self.pending[0]
                if not self.samples or self.samples[-1][0] < t:
                    if time.monotonic()-arrival < .3: break
                self.pending.popleft()
                pair = interpolate(list(self.samples),t)
                if pair is None:
                    self.get_logger().warning('No time-aligned IMU; comparison skipped')
                    self.last = None
                    self.total = [0.,0.]
                    continue
                i,g = pair
                if self.last:
                    if t <= self.last[0]: continue
                    if t-self.last[0] > .5:
                        self.total = [0.,0.]
                    else:
                        self.total[0] += wrap(i-self.last[1])
                        self.total[1] += wrap(o-self.last[2])
                self.last = (t,i,o)
                a,b = map(math.degrees,self.total)
                self.csv.writerow([t,i,o,g,a,b,b-a])
                self.file.flush()
                if time.monotonic()-self.last_print > 1:
                    self.get_logger().info(f'IMU turn={a:.2f} deg; odom turn={b:.2f} deg; difference={b-a:.2f} deg')
                    self.last_print = time.monotonic()
    rclpy.init()
    n = Compare()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        n.file.close()
        n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
