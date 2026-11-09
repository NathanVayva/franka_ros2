#!/usr/bin/env python3
"""Keyboard teleop for fr3_joint1 via keyboard_joint1_controller.

  q / d : turn joint 1 negative / positive (hold the key)
  z / s : increase / decrease max speed
  space : stop        Ctrl+C : quit
Stops by itself before the joint limit (SOFT_LIMIT).
"""
import select
import sys
import termios
import tty

import rclpy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# Tuning knobs (real robot: keep them low).
MAX_SPEED = 0.6      # rad/s, hard ceiling for z
ACCEL = 0.5          # rad/s^2, ramp so libfranka does not trip a reflex (its rate limiter is off)
RELEASE_TIMEOUT = 0.6  # s without key repeat = key released (covers the OS repeat delay)
RATE = 200.0         # Hz
SOFT_LIMIT = 2.75    # rad, FR3 joint1 hard limit is 2.9007: stay away (Gazebo sticks there, libfranka trips)


def ramp(current, target, max_step):
    return current + max(-max_step, min(max_step, target - current))


def braking_ok(pos, vel):
    """False if braking now at ACCEL would still end past SOFT_LIMIT."""
    return abs(pos + vel * abs(vel) / (2 * ACCEL)) < SOFT_LIMIT


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)  # keep context alive for the ramp-down
    node = rclpy.create_node('keyboard_joint1')
    pub = node.create_publisher(Float64MultiArray, '/keyboard_joint1_controller/commands', 10)
    state = {}
    node.create_subscription(JointState, '/joint_states',
                             lambda m: state.update(zip(m.name, m.position)), 10)
    speed, target, vel, idle = 0.2, 0.0, 0.0, 0.0
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin)
    print(__doc__)
    try:
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 1.0 / RATE)[0]:
                key = sys.stdin.read(1)
                idle = 0.0
                if key == 'q':
                    target = -speed
                elif key == 'd':
                    target = speed
                elif key in 'zs':
                    speed = min(MAX_SPEED, max(0.05, speed + (0.05 if key == 'z' else -0.05)))
                    print(f'\rvitesse max: {speed:.2f} rad/s   ', end='', flush=True)
                elif key == ' ':
                    target = 0.0
            else:
                idle += 1.0 / RATE
                if idle > RELEASE_TIMEOUT:
                    target = 0.0
            rclpy.spin_once(node, timeout_sec=0)
            pos = state.get('fr3_joint1')
            #print(f'\n\rPose: {pos} rads',end='',flush=True)
            if pos is None or (target * pos > 0 and not braking_ok(pos, vel)):
                target = 0.0  # no joint_states yet, or would overshoot the soft limit
            if target and not pub.get_subscription_count():
                print('\rkeyboard_joint1_controller inactif: lancez la sim avec '
                      'controller:=keyboard_joint1_controller', end='', flush=True)
            vel = ramp(vel, target, ACCEL / RATE)
            pub.publish(Float64MultiArray(data=[vel] + [0.0] * 6))
    except KeyboardInterrupt:
        pass
    finally:
        while abs(vel) > 1e-9:  # ramp down instead of a velocity step
            vel = ramp(vel, 0.0, ACCEL / RATE)
            pub.publish(Float64MultiArray(data=[vel] + [0.0] * 6))
            select.select([], [], [], 1.0 / RATE)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    if sys.argv[1:] == ['test']:
        assert ramp(0.0, 1.0, 0.1) == 0.1 and ramp(0.0, -1.0, 0.1) == -0.1 and ramp(0.95, 1.0, 0.1) == 1.0
        assert braking_ok(0.0, 0.6) and not braking_ok(2.5, 0.6) and braking_ok(2.5, -0.6)
        print('ok')
    else:
        main()
