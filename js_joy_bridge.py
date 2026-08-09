#!/usr/bin/env python3
"""Kernel /dev/input/js* → /joy 브리지 (값 가공 없음, 스케일만).

ROS joy_node(SDL) 가 안 먹을 때 / 부호 추측 remapping 없이 커널 값을 그대로 보냄.
joystick_teleop 이 기동 시 휴지값을 자동으로 맞춰 읽는다.

  python3 js_joy_bridge.py [/dev/input/js1]
"""
import os
import struct
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

JS_EVENT_FMT = 'IhBB'
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FMT)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80


class JsJoyBridge(Node):
    def __init__(self, device: str):
        super().__init__('js_joy_bridge')
        self.device = device
        self.pub = self.create_publisher(Joy, '/joy', 10)
        self.axes = {}
        self.buttons = {}
        self.fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK)
        self.get_logger().info(f'열린 장치: {device} → /joy (raw 커널 스케일)')
        self.create_timer(0.02, self._tick)

    def _publish(self):
        n_ax = max(list(self.axes.keys()) + [7], default=7) + 1
        n_btn = max(list(self.buttons.keys()) + [11], default=11) + 1
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.device
        msg.axes = [float(self.axes.get(i, 0.0)) for i in range(n_ax)]
        msg.buttons = [int(self.buttons.get(i, 0)) for i in range(n_btn)]
        self.pub.publish(msg)

    def _drain(self):
        while True:
            try:
                raw = os.read(self.fd, JS_EVENT_SIZE)
            except BlockingIOError:
                break
            if len(raw) < JS_EVENT_SIZE:
                break
            _t, value, etype, number = struct.unpack(JS_EVENT_FMT, raw)
            is_init = bool(etype & JS_EVENT_INIT)
            etype &= ~JS_EVENT_INIT
            if etype == JS_EVENT_AXIS:
                # 커널 raw 그대로. 휴지/당김 부호는 teleop 가 자동 맞춤.
                self.axes[number] = max(-1.0, min(1.0, value / 32767.0))
            elif etype == JS_EVENT_BUTTON:
                self.buttons[number] = 1 if value else 0
            if is_init:
                pass  # 초기값도 그대로 반영

    def _tick(self):
        self._drain()
        # 이벤트 전에도 0축 발행 → teleop 자동 캘리 샘플이 쌓이게
        self._publish()


def main():
    dev = sys.argv[1] if len(sys.argv) > 1 else '/dev/input/js1'
    if not os.path.exists(dev):
        for cand in ('/dev/input/js0', '/dev/input/js1'):
            if os.path.exists(cand):
                dev = cand
                break
        else:
            print(f'조이 장치 없음: {dev}', file=sys.stderr)
            sys.exit(1)

    rclpy.init()
    node = JsJoyBridge(dev)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            os.close(node.fd)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
