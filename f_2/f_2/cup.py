#!/usr/bin/env python3
import rclpy
import DR_init
import time
from  math import *
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
            movec,
            # movesx,  # We won't use movesx
            set_singular_handling,
            set_velj,
            set_accj,
            set_velx,
            set_accx,
            trans,
            DR_AXIS_Z,
            DR_AXIS_Y,
            DR_FC_MOD_REL,
            DR_MV_ORI_RADIAL,
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
            #print(f"Waiting for digital input #{sig_num} to be {desired_state}")

    def gripper_grip():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        #node.get_logger().info("Waiting for gripper to close...")
        rclpy.spin_once(node, timeout_sec=0.5)
        wait_digital_input(sig_num=1, desired_state=True)
        node.get_logger().info("Gripper closed")
        time.sleep(0.2)

    def gripper_release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        #node.get_logger().info("Waiting 0.2s for release...")
        rclpy.spin_once(node, timeout_sec=0.2)
        wait_digital_input(sig_num=2, desired_state=True)
        node.get_logger().info("Gripper opened")
        time.sleep(0.2)

    def gripper_measure():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        node.get_logger().info("Measure...")
        #rclpy.spin_once(node, timeout_sec=0.5)
        #wait_digital_input(sig_num=1, desired_state=True)
        node.get_logger().info("Gripper closed")
        time.sleep(0.3)

    #### 컵 하나 집기    
    def grip_cup(cup_index, last_pose = [366.998, 6.125, 194.183, 3.263, -179.907, 3.271]):
        node.get_logger().info(f"Start gripping cup. Cup index: {cup_index}")
        gripper_release()
        time.sleep(0.2)

        decreased_height_grip = [0, 0, (-1) * CUP_STACK_GAP * cup_index, 0, 0, 0] # 컵 집는 위치
        cup_gripping_point = [a + b for a, b in zip(cup_starting_point_top, decreased_height_grip)]
        z_up_3 = [0, 0, 30, 0, 0, 0]
        cup_gripping_point_up = [a + b for a, b in zip(cup_gripping_point, z_up_3)]


        # movesx([
        #     posx(last_pose),
        #     posx(cup_gripping_point_up),
        #     posx(cup_gripping_point)
        # ], ref=0)
        
        movel(posx(last_pose))
        movel(posx(cup_gripping_point_up))
        movel(posx(cup_gripping_point))

        node.get_logger().info(f"Move to picking place: {cup_gripping_point}")
        time.sleep(0.2)
        gripper_grip()
        z_up_11 = [0, 0, 130, 0, 0, 0]
        cup_gripping_point_above = [a + b for a, b in zip(cup_gripping_point, z_up_11)]

        return cup_gripping_point_above
    
    ######################################################################
    # 좌표 정의
    ######################################################################
    # 홈
    Global_0 = posj(0.00, 0.0, 90.0, 0.0, 90.0, 0.0)

    
    # 컵 시작 위치, 꼭대기 (11층) 집는 위치 (베이스 좌표)
    cup_starting_point_top = [405.801, 222.796, 213.67, 90.0, 180.0, 90.0]

    CUP_DIAMETER = 86
    CUP_STACK_GAP = 11.5
    root3 = sqrt(3)


    # 로봇 설정
    set_singular_handling()  # or pass a parameter if needed
    set_velj(50.0)
    set_accj(50.0)
    set_velx(100.0, 60.625)
    set_accx(100.0, 50.5)

    cup_index = 0
    ###################### 시작
    #while rclpy.ok():
    put_down_up = [366.998, 6.125, 194.183, 3.263, -179.907, 3.271]
    starting_point = [700.0, -100.0, 84, 90.0, 180.0, 90.0]
    gripper_release()
    time.sleep(0.2)
    movej(Global_0)
    
    # 컵 쌓기
    for z in range(3):
        z_add = [0, 0, 94 * z, 0, 0, 0]
        z_value = [a + b for a, b in zip(starting_point, z_add)]
        for x in range(3-z):
            x_add = [(-1) * CUP_DIAMETER  * (root3 / 3) * z + (-1) * CUP_DIAMETER  * (root3 / 2) * x, 0, 0, 0, 0, 0]
            xz_value = [a + b for a, b in zip(z_value, x_add)]
            for y in range(1+x):
                #gripper_release()
                node.get_logger().info(f"floor: {z}, x: {x}, y: {y}")
                y_add = [0, CUP_DIAMETER/2 * x + (-1) * CUP_DIAMETER  * y, 0, 0, 0, 0]
                xyz_value = [a + b for a, b in zip(xz_value, y_add)]

                # 컵 하나 가져오기
                last_pose = grip_cup(cup_index, put_down_up)

                z_up_11 = [0, 0, 110, 0, 0, 0]
                xyz_value_up = [a + b for a, b in zip(xyz_value, z_up_11)]
                # movesx([
                #     posx(last_pose),
                #     posx(xyz_value_up),
                #     posx(xyz_value)
                # ], ref=0)
                
                movel(last_pose)
                node.get_logger().info(f"Move to: {xyz_value_up}")
                movel(xyz_value_up)
                node.get_logger().info(f"Move to: {xyz_value}")
                movel(xyz_value)
                put_down_up = xyz_value_up

                cup_index += 1
                
    gripper_release()
    time.sleep(0.2)
    movej(Global_0)

    node.get_logger().info("Shutting down node.")
    rclpy.shutdown()

if __name__ == "__main__":
    main()






