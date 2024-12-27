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
            task_compliance_ctrl,
            set_desired_force,
            check_force_condition,
            release_compliance_ctrl,
            get_current_posx,
            movej,
            movel,
             movesx,
            # movesx,  # We won't use movesx
            set_singular_handling,
            set_velj,
            set_accj,
            set_velx,
            set_accx,
            trans,
            DR_AXIS_Z,
            DR_FC_MOD_REL,
        )
        from DR_common2 import (posj, posx)

    except ImportError as e:
        node.get_logger().error(f"Error importing Doosan libraries: {e}")
        return


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
        time.sleep(0.2)

    def gripper_release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        node.get_logger().info("Waiting 0.1s for release...")
        rclpy.spin_once(node, timeout_sec=0.1)
        wait_digital_input(sig_num=2, desired_state=True)
        node.get_logger().info("Gripper opened")
        time.sleep(0.2)

    def gripper_measure():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        node.get_logger().info("Measure...")
        rclpy.spin_once(node, timeout_sec=0.5)
        #wait_digital_input(sig_num=1, desired_state=True)
        node.get_logger().info("Gripper closed")
        time.sleep(0.3)
    
            

    Global_0 = posj(0.00, -0.01, 90.02, -0.01, 89.99, 0.01)

    set_singular_handling()  # or pass a parameter if needed
    set_velj(50.0)
    set_accj(50.0)
    set_velx(100.0, 30.625)
    set_accx(100.0, 50.5)

    # 가장 우측 도미노 집는 위치
    domino_starting_point = posx([350.0, -230.0, 55, 30, -180, 30])
    right_start_point = posx([256.882, -234.63, 53.755, 30.124, -178.73, 29.768])


    domino_length = 0
    domino_point = []
    while rclpy.ok():
        node.get_logger().info("Starting Domino ...")
        gripper_release()
        time.sleep(1)
        movej(Global_0)
        time.sleep(1)
        
        domino_count = 50
        for i in range(domino_count):
            
            #### 도미노 하나 짚기
            delta = [0, 0, 100, 0, 0, 0]
            alpha = [0, i * 16.60, 0, 0, 0, 0]
            right_point = trans(right_start_point, alpha)
            #node.get_logger().info(f"연산 : {right_point}, 타입: {type(right_point)}\n")
            right_point = right_point.tolist()
            right_point_up = trans(posx(right_point), delta)
            right_point_up= right_point_up.tolist()
            movesx([
                posx(right_point_up),
                posx(right_point)
            ], ref=0)

            gripper_grip()

            if domino_length <= 2:
                gamma = [0, i * 40, 0, 0, 0, 0]
                domino_point = trans(domino_starting_point, gamma).tolist()
                domino_point_up = trans(posx(domino_point), delta).tolist()
                movesx([
                    posx(right_point_up ),
                    posx(domino_point_up),
                    posx(domino_point)
                ], ref=0)
            else:
                movesx([
                    posx(right_point_up ),
                    posx(domino_point_up)
                ], ref=0)
                theta = [20, 20, 0, 0, 0, 0]
                domino_point = trans(domino_point, theta).tolist()
                domino_point_up = trans(posx(domino_point), delta).tolist()
                movesx([
                    posx(domino_point_up),
                    posx(domino_point)
                ], ref=0)

            gripper_release()

            movel(posx(domino_point_up))

            domino_length += 1
    
        
        gripper_release()


        movej(Global_0)
        node.get_logger().info("One full cycle done! Looping again...\n")
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info("Shutting down node.")
    rclpy.shutdown()

if __name__ == "__main__":
    main()






