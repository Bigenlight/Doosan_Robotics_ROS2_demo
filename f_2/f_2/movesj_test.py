#!/usr/bin/env python3
import rclpy
from DSR_ROBOT2 import movej, movesj
from DR_common2 import posj

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("dsr_test")

    # 예: 현재 로봇 상태가 홈 위치가 아니라고 가정
    home = posj(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    q1   = posj(10.0, 20.0, 30.0, 40.0, 50.0, 60.0)
    q2   = posj(0.0, 30.0, 60.0, 20.0, 40.0, 50.0)

    # movej로 일단 홈으로 이동
    movej(home, vel=30, acc=60)

    # 직후 movesj로 q1 → q2 스플라인 이동
    qlist = [q1, q2]
    movesj(qlist, vel=20, acc=60)

    # 혹은 time기반
    # movesj(qlist, time=5.0)

    node.get_logger().info("Motion command sent.")

    # 조금 기다리거나 spin
    rclpy.spin_once(node, timeout_sec=3.0)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
