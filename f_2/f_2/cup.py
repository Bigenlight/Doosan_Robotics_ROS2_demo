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
    
    
    ######################################################################
    # 좌표 정의
    ######################################################################
    # 홈
    Global_0 = posj(0.00, 0.0, 90.0, 0.0, 90.0, 0.0)

    
    # 컵 시작 위치, 꼭대기 (11층) 집는 위치 (베이스 좌표)
    cup_starting_point_top = [405.801, 222.796, 213.67, 90.0, 180.0, 90.0]

    CUP_STACK_GAP = 11.5

    # 로봇 설정
    set_singular_handling()  # or pass a parameter if needed
    set_velj(50.0)
    set_accj(50.0)
    set_velx(100.0, 60.625)
    set_accx(100.0, 50.5)

    # 가장 우측 도미노 집는 위치
    domino_starting_point = posx([350.0, -230.0, 55, 30, -180, 30])
    right_start_point = posx([256.882, -234.63, 53.755, 30.124, -178.73, 29.768])
    
    cup_index = 0
    ###################### 시작
    while rclpy.ok() and cup_index < 11:
        gripper_release()
        time.sleep(1)
        movej(Global_0)
        time.sleep(1)

        #result = [a + b for a, b in zip(list1, list2)]


        #### 컵 하나 집기
        decreased_height_grip = [0, 0, -1 * CUP_STACK_GAP * cup_index, 0, 0, 0] # 컵 집는 위치
        cup_gripping_point = [a + b for a, b in zip(cup_starting_point_top, decreased_height_grip)]
        z_up_3 = [0, 0, 30, 0, 0, 0]
        cup_gripping_point_up = [a + b for a, b in zip(cup_gripping_point, z_up_3)]

        movesx([
            posx(cup_gripping_point_up),
            posx(cup_gripping_point)
        ], ref=0)

        gripper_grip()
        
        z_up_11 = [0, 0, 110, 0, 0, 0]
        cup_gripping_point_above = [a + b for a, b in zip(cup_gripping_point, z_up_11)]
        movel(
            posx(cup_gripping_point_above)
        )

        movej(Global_0)


### 이전 코드
        # right_start_point = posx(right_start_point)
        # node.get_logger().info(f"감지 위치 {right_start_point}")
        # node.get_logger().info("Starting Domino ...")
        # gripper_release()




        # for i in range(11): # 0~10
            
        #     ### 끝내기
        #     if domino_length >= end_count or domino_length >= domino_count:
        #         omega = [0, -40, 0, 0, 0, 0]
        #         break_point = trans(domino_starting_point, omega).tolist()
        #         omega = [0, 40, 0, 0, 0, 0]
        #         break_point_go = trans(posx(domino_starting_point), omega).tolist()
        #         delta = [0, 0, 100, 0, 0, 0]
        #         break_point_up = trans(domino_starting_point, delta).tolist()

        #         movel(
        #             posx(domino_point_up),
        #         )
        #         gripper_measure()
        #         movesx([
        #             posx(break_point_up),
        #             posx(break_point),
        #             posx(break_point_go),
        #         ], ref=0, vel = 200)
        #         gripper_release()
        #         break


        #     #### 도미노 하나 짚기
        #     delta = [0, 0, 100, 0, 0, 0]
        #     alpha = [0, domino_length * BLOCK_THICKNESS_MM + 10, 0, 0, 0, 0]
        #     right_point = trans(right_start_point, alpha)
        #     right_point = right_point.tolist()
        #     right_point_up = trans(posx(right_point), delta)
        #     right_point_up= right_point_up.tolist()
        #     movesx([
        #         posx(right_point_up),
        #         posx(right_point)
        #     ], ref=0)

        #     gripper_grip()


        #     if domino_length > starting_circle and theta_count < 6 and rotate_flag == True:
        #         node.get_logger().info(f"회전 중, 현대 회전 카운트 : {theta_count}\n")
        #         theta = [
        #             [5, 40, 0, 0, 0, -30],#1
        #             [30, 70, 0, 0, 0, -60],#2
        #             [70, 80, 0, 0, 0, -90],#3
        #             [120, 70, 0, 0, 0, 60],#4
        #             [140, 40, 0, 0, 0, 30],#5
        #             [145, 0, 0, 0, 0, 0],#6
        #         ]
        #         domino_point_circle = trans(domino_point, theta[theta_count]).tolist()
        #         domino_point_circle_up = trans(posx(domino_point_circle), delta).tolist()
        #         movesx([
        #             posx(right_point_up ),
        #             posx(domino_point_circle_up),
        #             posx(domino_point_circle)
        #         ], ref=0)
        #         gripper_release()
        #         movel(
        #             posx(domino_point_circle_up),
        #         )
        #         theta_count += 1
        #         if theta_count >= 6:
        #             dir_flag = -1
        #             new_domino_starting_point = posx(domino_point_circle)
        #             node.get_logger().info(f"시작 포인트 변경 : {new_domino_starting_point}\n")
        #             theta_count = 0
        #             rotate_flag = False

        #     elif rotate_flag == True:
        #         node.get_logger().info(f"우에서 좌\n")
        #         gamma = [0, domino_length * 50, 0, 0, 0, 0]
        #         domino_point = trans(domino_starting_point, gamma).tolist()
        #         domino_point_up = trans(posx(domino_point), delta).tolist()
        #         movesx([
        #             posx(right_point_up ),
        #             posx(domino_point_up),
        #             posx(domino_point)
        #         ], ref=0)
        #         gripper_release()
        #         movel(
        #             posx(domino_point_up),
        #         )
        #     elif rotate_flag == False:
        #         node.get_logger().info(f"좌에서 우\n")
        #         gamma = [0, i * (-50), 0, 0, 0, 0]
        #         domino_point = trans(new_domino_starting_point, gamma).tolist()
        #         domino_point_up = trans(posx(domino_point), delta).tolist()
        #         movesx([
        #             posx(right_point_up ),
        #             posx(domino_point_up),
        #             posx(domino_point)
        #         ], ref=0)
        #         gripper_release()
        #         movel(
        #             posx(domino_point_up),
        #         )
        #         i += 1



        #     gripper_release()


        cup_index += 1

        
        gripper_release()


    movej(Global_0)
    node.get_logger().info("One full cycle done! Looping again...\n")
    rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info("Shutting down node.")
    rclpy.shutdown()

if __name__ == "__main__":
    main()






