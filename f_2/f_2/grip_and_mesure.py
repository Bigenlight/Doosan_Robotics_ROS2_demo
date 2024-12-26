#!/usr/bin/env python3
import rclpy
import DR_init
import time

ROBOT_ID    = "dsr01"
ROBOT_MODEL = "m0609"
ON = 1
OFF = 0

DR_init.__dsr__id    = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("doosan_drl_example", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            set_digital_output,
            get_digital_input,
            movej,
            movel,
            # movesx,  # We won't use movesx
            set_singular_handling,
            set_velj,
            set_accj,
            set_velx,
            set_accx,
            trans,
        )
        from DR_common2 import (posj, posx)

    except ImportError as e:
        node.get_logger().error(f"Error importing Doosan libraries: {e}")
        return

    # For a 3x3 pattern => 9 positions
    def get_pattern_point_3x3(c0, c1, c2, c3, index):
        row = index // 3
        col = index % 3

        p0 = c0[:]
        p1 = c1[:]
        p2 = c2[:]
        p3 = c3[:]

        fx = col / 2.0
        fy = row / 2.0

        top    = [p0[i] + fx*(p1[i]-p0[i]) for i in range(6)]
        bottom = [p3[i] + fx*(p2[i]-p3[i]) for i in range(6)]
        final  = [top[i] + fy*(bottom[i]-top[i]) for i in range(6)]

        return posx(final)

    def wait_digital_input(sig_num, desired_state=True, period=0.2):
        while rclpy.ok():
            val = get_digital_input(sig_num)
            if val == desired_state:
                break
            time.sleep(period)
            print(f"Waiting for digital input #{sig_num} to be {desired_state}")

    def gripper_grip():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        node.get_logger().info("Waiting for gripper to close...")
        rclpy.spin_once(node, timeout_sec=0.5)
        wait_digital_input(sig_num=1, desired_state=True)
        node.get_logger().info("Gripper closed")
        time.sleep(0.5)

    def gripper_release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        node.get_logger().info("Waiting 0.1s for release...")
        rclpy.spin_once(node, timeout_sec=0.1)
        wait_digital_input(sig_num=2, desired_state=True)
        node.get_logger().info("Gripper opened")
        time.sleep(0.5)


    Global_0 = posj(0.00, -0.01, 90.02, -0.01, 89.99, 0.01)

    set_singular_handling()  # or pass a parameter if needed
    set_velj(50.0)
    set_accj(50.0)
    set_velx(70.0, 30.625)
    set_accx(100.0, 50.5)

    while rclpy.ok():
        node.get_logger().info("grip & release")
        gripper_release()
        time.sleep(1)
        gripper_grip()


    node.get_logger().info("Shutting down node.")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
