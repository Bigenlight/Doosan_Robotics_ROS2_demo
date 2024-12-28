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
    # 4. 힘 제어를 이용한 길이 측정 (오른쪽 → +Y만 사용)
    ######################################################################
    def measure_length_with_force_control():
        """
        - 오른쪽 상단(오버헤드) 좌표 -> 오른쪽 실제 측정 좌표 이동
        - +Y 힘 제어로 이동 후, 최종 위치를 읽음
        - '왼쪽편' 고정 좌표의 Y값과 비교하여 이동 거리(mm) -> 블록 개수 산출
        """

        node.get_logger().info("==> 길이 측정 알고리즘 시작")

        # (A) 초기 자세 -> 오른쪽 상단 -> 오른쪽 측정 지점
        movej(Global_0)
        node.get_logger().info("[1] 초기 자세로 이동 완료.")
        time.sleep(0.5)

        movel(right_over_pose)
        node.get_logger().info("[2] 오른쪽 상단(오버헤드) 좌표로 이동 완료.")

        movel(right_pose)
        node.get_logger().info("[3] 오른쪽 실제 측정 좌표로 이동 완료.")

        # (B) +Y 힘 제어
        node.get_logger().info("[4] +Y 힘 제어를 시작합니다.")
        task_compliance_ctrl(stx=[3000, 3000, 3000, 100, 100, 100])

        # 예시: 15N 힘, 2~3N 충돌 감지 시 멈춤
        set_desired_force(fd=[0, 20, 0, 0, 0, 0], 
                          dir=[0, 1, 0, 0, 0, 0], 
                          mod=DR_FC_MOD_REL)
        node.get_logger().info("로봇이 +Y 방향으로 이동 중...")

        # Y축 힘이 2.5N 이상이면 충돌로 보고 멈춘다(예시)
        while not check_force_condition(DR_AXIS_Y, max=10.0):
            time.sleep(0.05)

        # 최종 위치 (힘으로 멈춘 지점)
        final_pose_data, _ = get_current_posx()
        node.get_logger().info(f"[5] +Y 힘 제어 종료. 최종 위치: {final_pose_data}")

        release_compliance_ctrl()

        # (C) 길이 계산
        # '왼쪽편' 고정 좌표와의 Y값 차이 => mm 단위
        #   예) left_fixed_pose_list[1] (왼쪽 Y) - final_pose_data[1] (오른쪽측 최종 Y)
        #   혹은 절댓값(abs)으로 계산
        measured_distance_mm = abs(left_fixed_pose_list[1] - final_pose_data[1])
        node.get_logger().info(f"  - 최종 측정된 거리(mm): {measured_distance_mm:.2f}")

        # 블록 개수 = 거리 / 블록 두께(15.1976 mm)
        block_count = round(measured_distance_mm / BLOCK_THICKNESS_MM)
        
        node.get_logger().info("=================================")
        node.get_logger().info("힘 제어를 통한 길이 측정 결과")
        node.get_logger().info(f" - 이동 거리:    {measured_distance_mm:.2f} mm")
        node.get_logger().info(f" - 블록 두께:     {BLOCK_THICKNESS_MM:.2f} mm")
        node.get_logger().info(f" - 블록 개수:     {block_count} 개")
        node.get_logger().info("=================================")

        # (D) 안전 위해 초기 자세로 복귀
        movel(right_pose)       # 원래의 오른쪽 측정 좌표로 잠시 복귀
        movel(right_over_pose)  # 상단으로 이동
        #movej(Global_0)         # 최종적으로 초기 자세
        node.get_logger().info("[6] 측정 후 초기 자세 복귀 완료.")

        return final_pose_data, block_count
    
    

    Global_0 = posj(0.00, -0.01, 90.02, -0.01, 89.99, 0.01)

    ######################################################################
    # 좌표 정의
    ######################################################################
    # 오른쪽 상단(오버헤드) 좌표 (충돌 방지용, 예시로 높이를 충분히 높임)
    right_over_pose_list = [256.882, -254.63, 123.755, 30.124, -178.73, 29.768]

    # 오른쪽 측정 좌표 (힘 제어 시작점)
    right_pose_list = [256.882, -254.63, 53.755, 30.124, -178.73, 29.768]

    # "왼쪽편" 고정된 좌표 (질문에서 주어진 값)
    left_fixed_pose_list = [257.254, 173.072, 45.487, 161.038, -179.052, 161.521]
    # ※ 이 좌표로 직접 이동하지 않고, "기준값"으로만 사용

    # Doosan posx 형태로 변환
    right_over_pose = posx(right_over_pose_list)
    right_pose      = posx(right_pose_list)
    left_fixed_pose = posx(left_fixed_pose_list)

    set_singular_handling()  # or pass a parameter if needed
    set_velj(50.0)
    set_accj(50.0)
    set_velx(300.0, 60.625)
    set_accx(100.0, 50.5)

    BLOCK_THICKNESS_MM = 14.45
    # 가장 우측 도미노 집는 위치
    domino_starting_point = posx([350.0, -230.0, 55, 30, -180, 30])
    right_start_point = posx([256.882, -234.63, 53.755, 30.124, -178.73, 29.768])
    

    ###################### 시작

    domino_length = 0
    domino_point = []
    starting_circle = 8
    end_count = 25


    gripper_release()
    time.sleep(1)
    movej(Global_0)
    time.sleep(1)
    gripper_measure()

    right_start_point, domino_count = measure_length_with_force_control()

    right_start_point = posx(right_start_point)
    node.get_logger().info(f"감지 위치 {right_start_point}")
    node.get_logger().info("Starting Domino ...")
    gripper_release()


    theta_count = 0
    dir_flag = 1
    rotate_flag = True
    i = 1 
    while True:
        
        ### 끝내기
        if domino_length >= end_count or domino_length >= domino_count:
            omega = [0, -40, 0, 0, 0, 0]
            break_point = trans(domino_starting_point, omega).tolist()
            omega = [0, 40, 0, 0, 0, 0]
            break_point_go = trans(posx(domino_starting_point), omega).tolist()
            delta = [0, 0, 100, 0, 0, 0]
            break_point_up = trans(domino_starting_point, delta).tolist()

            movel(
                posx(domino_point_up),
            )
            gripper_measure()
            movesx([
                posx(break_point_up),
                posx(break_point),
                posx(break_point_go),
            ], ref=0, vel = 200)
            gripper_release()
            break


        #### 도미노 하나 짚기
        delta = [0, 0, 100, 0, 0, 0]
        alpha = [0, domino_length * BLOCK_THICKNESS_MM + 10, 0, 0, 0, 0]
        right_point = trans(right_start_point, alpha)
        right_point = right_point.tolist()
        right_point_up = trans(posx(right_point), delta)
        right_point_up= right_point_up.tolist()
        movesx([
            posx(right_point_up),
            posx(right_point)
        ], ref=0)

        gripper_grip()


        if domino_length > starting_circle and theta_count < 6 and rotate_flag == True:
            node.get_logger().info(f"회전 중, 현대 회전 카운트 : {theta_count}\n")
            theta = [
                [5, 40, 0, 0, 0, -30],#1
                [30, 70, 0, 0, 0, -60],#2
                [70, 80, 0, 0, 0, -90],#3
                [120, 70, 0, 0, 0, 60],#4
                [140, 40, 0, 0, 0, 30],#5
                [145, 0, 0, 0, 0, 0],#6
            ]
            domino_point_circle = trans(domino_point, theta[theta_count]).tolist()
            domino_point_circle_up = trans(posx(domino_point_circle), delta).tolist()
            movesx([
                posx(right_point_up ),
                posx(domino_point_circle_up),
                posx(domino_point_circle)
            ], ref=0)
            gripper_release()
            movel(
                posx(domino_point_circle_up),
            )
            theta_count += 1
            if theta_count >= 6:
                dir_flag = -1
                new_domino_starting_point = posx(domino_point_circle)
                node.get_logger().info(f"시작 포인트 변경 : {new_domino_starting_point}\n")
                theta_count = 0
                rotate_flag = False

        elif rotate_flag == True:
            node.get_logger().info(f"우에서 좌\n")
            gamma = [0, domino_length * 50, 0, 0, 0, 0]
            domino_point = trans(domino_starting_point, gamma).tolist()
            domino_point_up = trans(posx(domino_point), delta).tolist()
            movesx([
                posx(right_point_up ),
                posx(domino_point_up),
                posx(domino_point)
            ], ref=0)
            gripper_release()
            movel(
                posx(domino_point_up),
            )
        elif rotate_flag == False:
            node.get_logger().info(f"좌에서 우\n")
            gamma = [0, i * (-50), 0, 0, 0, 0]
            domino_point = trans(new_domino_starting_point, gamma).tolist()
            domino_point_up = trans(posx(domino_point), delta).tolist()
            movesx([
                posx(right_point_up ),
                posx(domino_point_up),
                posx(domino_point)
            ], ref=0)
            gripper_release()
            movel(
                posx(domino_point_up),
            )
            i += 1



        gripper_release()


        domino_length += 1

    
    gripper_release()


    movej(Global_0)
    node.get_logger().info("One full cycle done! Looping again...\n")
    rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info("Shutting down node.")
    rclpy.shutdown()

if __name__ == "__main__":
    main()






