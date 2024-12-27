#!/usr/bin/env python3
import rclpy
import DR_init
import time
import math

ROBOT_ID    = "dsr01"
ROBOT_MODEL = "m0609"
ON = 1
OFF = 0

# 두산 로보틱스 설정
DR_init.__dsr__id    = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def main(args=None):
    ######################################################################
    # 1. ROS2 Node 초기화
    ######################################################################
    rclpy.init(args=args)
    node = rclpy.create_node("doosan_drl_block_measure", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            movej,
            movel,
            trans,
            get_current_posx,
            set_velj,
            set_accj,
            set_velx,
            set_accx,
            task_compliance_ctrl,
            set_desired_force,
            check_force_condition,
            release_compliance_ctrl,
            set_singular_handling,
            set_digital_output,
            get_digital_input,

            posj,
            posx,
            DR_AXIS_Y,
            DR_FC_MOD_REL,
        )
    except ImportError as e:
        node.get_logger().error(f"두산 라이브러리 임포트 에러: {e}")
        return

    ######################################################################
    # 2. 기본 세팅
    ######################################################################
    # 초기 자세 (예시)
    Global_0 = posj(0.00, -0.01, 90.02, -0.01, 89.99, 0.01)

    # 로봇 속도/가속도 세팅 (필요 시 조정)
    set_velj(50.0)
    set_accj(50.0)
    set_velx(50.0, 50.625)
    set_accx(50.0, 50.5)

    # 오른쪽, 왼쪽 좌표 (질문에서 제시된 값)
    right_pose_list = [256.882, -254.63, 53.755, 30.124, -178.73, 29.768]
    left_pose_list  = [274.499,  261.32, 51.196, 145.692, -175.886, 139.237]

    # Doosan posx 형태로 감싸기
    right_pose = posx(right_pose_list)
    left_pose  = posx(left_pose_list)

    # 나무 블록 두께(1 cm)
    BLOCK_THICKNESS_CM = 1.0

    ######################################################################
    # 3. 디지털 IO 제어 및 그리퍼 함수
    ######################################################################
    def wait_digital_input(sig_num, desired_state=True, period=0.2):
        while rclpy.ok():
            val = get_digital_input(sig_num)
            if val == desired_state:
                break
            time.sleep(period)
            node.get_logger().info(f"Waiting for digital input #{sig_num} to be {desired_state}")

    def gripper_grip():
        """
        그리퍼를 완전히 닫는 함수
        """
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        node.get_logger().info("Waiting for gripper to close...")
        rclpy.spin_once(node, timeout_sec=0.5)
        wait_digital_input(sig_num=1, desired_state=True)
        node.get_logger().info("Gripper closed")
        time.sleep(0.2)

    ######################################################################
    # 4. 보조 함수: 힘 제어를 이용한 길이 측정
    ######################################################################
    def measure_length_with_force_control():
        """
        1) 오른쪽 좌표에서 +Y 방향으로 힘 제어를 하여 길이 측정
        2) 왼쪽 좌표에서 -Y 방향으로 힘 제어를 하여 길이 측정
        3) 두 값을 합산하여 총 길이 산출 후, 블록 개수 계산
        """
        node.get_logger().info("길이 측정을 시작합니다.")
        # 초기 자세로 이동
        movej(Global_0)
        node.get_logger().info("초기 자세로 이동 완료.")
        time.sleep(1)

        ##################################################################
        # [1] 오른쪽 위치 → +Y 축 힘 제어
        ##################################################################
        movel(right_pose)
        node.get_logger().info("오른쪽 위치로 이동 완료.")

        # z축 7cm (70mm) 상승하여 충돌 방지
        delta_up = [0, 0, 70, 0, 0, 0]
        approach_pose_right = trans(right_pose, delta_up)
        movel(approach_pose_right)
        node.get_logger().info("오른쪽 접근 자세로 이동 완료.")

        # +Y 방향(양의 Y축)으로 힘 제어
        # 예: 30N을 가하면서 이동, Y축 힘이 일정 기준(여기서는 5N) 이상 느껴지면 멈춤
        task_compliance_ctrl(stx=[3000, 3000, 3000, 100, 100, 100])
        set_desired_force(fd=[0, 1, 0, 0, 0, 0], dir=[0, 1, 0, 0, 0, 0], mod=DR_FC_MOD_REL)
        node.get_logger().info("힘 제어(+Y)로 이동 중...")

        # 힘 조건이 만족될 때까지 대기
        while not check_force_condition(DR_AXIS_Y, max=5):
            time.sleep(0.1)

        # 현재 위치 측정
        current_pose_r2l = get_current_posx()
        pose_r2l, status_r2l = current_pose_r2l
        node.get_logger().info(f"오른쪽 → 왼쪽(+Y) 힘 제어 후 현재 위치: {pose_r2l}")

        # 힘 제어 해제
        release_compliance_ctrl()

        # +Y 이동 거리 계산 (mm → cm)
        distance_r2l_mm = abs(pose_r2l[1] - right_pose_list[1])
        distance_r2l_cm = distance_r2l_mm / 10.0
        node.get_logger().info(f"오른쪽 → 왼쪽(+Y) 이동 거리: {distance_r2l_cm:.2f} cm")

        ##################################################################
        # [2] 왼쪽 위치 → -Y 축 힘 제어
        ##################################################################
        movel(left_pose)
        node.get_logger().info("왼쪽 위치로 이동 완료.")

        # z축 7cm (70mm) 상승하여 충돌 방지
        approach_pose_left = trans(left_pose, delta_up)
        movel(approach_pose_left)
        node.get_logger().info("왼쪽 접근 자세로 이동 완료.")

        # -Y 방향(음의 Y축)으로 힘 제어
        task_compliance_ctrl(stx=[3000, 3000, 3000, 100, 100, 100])
        set_desired_force(fd=[0, -1, 0, 0, 0, 0], dir=[0, 1, 0, 0, 0, 0], mod=DR_FC_MOD_REL)
        node.get_logger().info("힘 제어(-Y)로 이동 중...")

        # 힘 조건이 만족될 때까지 대기
        while not check_force_condition(DR_AXIS_Y, max=5):
            time.sleep(0.1)

        # 현재 위치 측정
        current_pose_l2r = get_current_posx()
        pose_l2r, status_l2r = current_pose_l2r
        node.get_logger().info(f"왼쪽 → 오른쪽(-Y) 힘 제어 후 현재 위치: {pose_l2r}")

        # 힘 제어 해제
        release_compliance_ctrl()

        # -Y 이동 거리 계산 (mm → cm)
        distance_l2r_mm = abs(pose_l2r[1] - left_pose_list[1])
        distance_l2r_cm = distance_l2r_mm / 10.0
        node.get_logger().info(f"왼쪽 → 오른쪽(-Y) 이동 거리: {distance_l2r_cm:.2f} cm")

        ##################################################################
        # [3] 총 길이 및 블록 개수 계산
        ##################################################################
        total_distance_cm = distance_r2l_cm + distance_l2r_cm
        block_count = round(total_distance_cm / BLOCK_THICKNESS_CM)  # 반올림하여 정수화

        node.get_logger().info("=================================")
        node.get_logger().info("힘 제어를 통한 총 길이 측정 완료")
        node.get_logger().info(f" - 오른쪽(+Y) 측정 거리: {distance_r2l_cm:.2f} cm")
        node.get_logger().info(f" - 왼쪽(-Y) 측정 거리:  {distance_l2r_cm:.2f} cm")
        node.get_logger().info(f" - 총 거리:             {total_distance_cm:.2f} cm")
        node.get_logger().info(f" - 나무 블록 개수:      {block_count} 개 (두께 1cm 기준)")
        node.get_logger().info("=================================")

        # 초기 자세로 복귀
        movel(approach_pose_right)  # 다시 오른쪽 접근 자세로 이동 (또는 안전한 자세)
        movej(Global_0)
        node.get_logger().info("초기 자세로 복귀 완료.")

    ######################################################################
    # 5. 노드 실행 시 동작 (메인 로직)
    ######################################################################
    try:
        # 노드 실행 직후, 그리퍼를 닫은 상태로 시작
        gripper_grip()
        node.get_logger().info("노드 시작 시 그리퍼가 닫힌 상태로 초기화했습니다.")

        # 메인 동작: 블록 길이 측정
        node.get_logger().info("나무 블록의 길이를 측정하고자 합니다.")
        measure_length_with_force_control()

        node.get_logger().info("측정을 완료하였으므로 노드를 종료합니다.")
        rclpy.shutdown()

    except KeyboardInterrupt:
        node.get_logger().info("사용자 종료 요청(Ctrl+C) 감지. 노드를 종료합니다.")
        rclpy.shutdown()

######################################################################
# 보조 함수: 디지털 입력 가져오기 (실제 하드웨어에 맞게 수정 필요)
######################################################################
def get_digital_input(sig_num):
    """
    실제 환경에서는 하드웨어와 통신하여 디지털 입력을 읽어와야 합니다.
    여기서는 예시로 항상 True/False를 단순 반환하도록 구성합니다.
    """
    # 사용자 환경에 맞게 수정 필요
    if sig_num == 1:
        return True
    return False
