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

    # Right pallet corners
    pr00 = posx([500.0, -1.51,   21.61, 86.72, 179.94, 86.74])
    pr02 = posx([500.0, -103.51, 21.61, 86.72, 179.94, 86.74])
    pr22 = posx([398.0, -103.51, 21.61, 86.72, 179.94, 86.74])
    pr20 = posx([398.0, -1.51,   21.61, 86.72, 179.94, 86.74])

    # Left pallet corners
    pl00 = posx([500.0, 147.0,  21.61, 6.55,  -179.64, 6.31])
    pl02 = posx([500.0, 45.0,   21.61, 6.55,  -179.64, 6.31])
    pl22 = posx([398.0, 45.0,   21.61, 6.55,  -179.64, 6.31])
    pl20 = posx([398.0, 147.0,  21.61, 6.55,  -179.64, 6.31])

    switch_dir_right =  posx([637.261, 21.246, 102.717, 91.62, 90.867, -89.742])
    switch_dir_left =  posx([571.379, -11.415, 35.363, 90.057, -94.83, -93.092])
    switch_dir_left_j1 = posj([19.256, 67.053, 4.966, 102.548, -119.048, -203.422])
    switch_dir_left_j2 = posj([29.483, 75.225, 39.167, 101.148, -116.943, -152.729])
    
    switch_dir_left_j3 = posj([8.789, 35.952, 39.319, 64.225, 75.029, -209.713])

    
    total_count = 9  # 3x3

    while rclpy.ok():
        node.get_logger().info("Starting a pick-and-place cycle from Right Pallet -> Left Pallet...")
        gripper_release()
        movej(Global_0)

        # Right -> Left
        for pallet_index in range(total_count):
            node.get_logger().info(f"[Right -> Left] Picking object at index: {pallet_index}")
            Pallet_Pose_r = get_pattern_point_3x3(pr00, pr02, pr22, pr20, pallet_index)

            # Approach from above
            delta = [0, 0, 70, 0, 0, 0]
            Pallet_Pose_r_up = trans(Pallet_Pose_r, delta)
            movel(Pallet_Pose_r_up)
            movel(Pallet_Pose_r)

            # Instead of movesx(...)
            # movesx([Pallet_Pose_r_up, Pallet_Pose_r], ref=0)
            # we do:
            #   movel(Pallet_Pose_r_up)
            #   movel(Pallet_Pose_r)

            gripper_grip()

            # Move to left side
            Pallet_Pose_l = get_pattern_point_3x3(pl00, pl02, pl22, pl20, pallet_index)
            Pallet_Pose_l_up = trans(Pallet_Pose_l, delta)

            
            # Instead of movesx([Pallet_Pose_r_up, Pallet_Pose_l_up], ref=0)
            # do two movel() calls or none if you prefer going directly:
            movel(Pallet_Pose_r_up)

            ###### 뒤집기
            switch_dir_right_up = trans(switch_dir_right, delta)
            movel(switch_dir_right_up)
            movel(switch_dir_right)
            gripper_release()
            movel(switch_dir_right_up)

            # switch_dir_left_up = trans(switch_dir_left, delta)
            # movel(switch_dir_left_up)
            # movel(switch_dir_left)
            movej(switch_dir_left_j1)
            movej(switch_dir_left_j2)
            gripper_grip()
            movej(switch_dir_left_j1)
            movej(switch_dir_left_j3)
            movej(Global_0)
            #######

            movel(Pallet_Pose_l_up)

            alpha = [0, 0, 10, 0, 0, 0]
            Pallet_Pose_l_release = trans(Pallet_Pose_l, alpha)
            movel(Pallet_Pose_l_release)
            gripper_release()
            movel(Pallet_Pose_l_up)

        movej(Global_0)
        node.get_logger().info("Starting a pick-and-place cycle from Left Pallet -> Right Pallet...")

        # Left -> Right
        for pallet_index in range(total_count):
            node.get_logger().info(f"[Left -> Right] Picking object at index: {pallet_index}")
            Pallet_Pose_l = get_pattern_point_3x3(pl00, pl02, pl22, pl20, pallet_index)
            delta = [0, 0, 70, 0, 0, 0]
            Pallet_Pose_l_up = trans(Pallet_Pose_l, delta)

            movel(Pallet_Pose_l_up)
            movel(Pallet_Pose_l)
            gripper_grip()
            movel(Pallet_Pose_l_up)

            Pallet_Pose_r = get_pattern_point_3x3(pr00, pr02, pr22, pr20, pallet_index)
            Pallet_Pose_r_up = trans(Pallet_Pose_r, delta)
            movel(Pallet_Pose_r_up)

            alpha = [0, 0, 10, 0, 0, 0]
            Pallet_Pose_r_release = trans(Pallet_Pose_r, alpha)
            movel(Pallet_Pose_r_release)
            gripper_release()
            movel(Pallet_Pose_r_up)

        movej(Global_0)
        node.get_logger().info("One full cycle done! Looping again...\n")
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info("Shutting down node.")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
