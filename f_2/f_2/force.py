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

    ######################################################################
    # 나무 블록 두께(16.75 mm)
    ######################################################################
    BLOCK_THICKNESS_MM = 16.75

    ######################################################################
    # 3. 디지털 IO 및 그리퍼 함수
    ######################################################################
    def wait_digital_input(sig_num, desired_state=True, period=0.2):
        """
        디지털 입력이 desired_state 상태가 될 때까지 대기
        """
        while rclpy.ok():
            val = get_digital_input(sig_num)
            if val == desired_state:
                break
            time.sleep(period)
            node.get_logger().info(f"Waiting for digital input #{sig_num} to be {desired_state}")

    def gripper_release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        node.get_logger().info("Gripper releasing...")
        rclpy.spin_once(node, timeout_sec=0.1)
        wait_digital_input(sig_num=2, desired_state=True)
        node.get_logger().info("Gripper opened")
        time.sleep(0.2)

    def gripper_grip():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        node.get_logger().info("Gripper closing...")
        rclpy.spin_once(node, timeout_sec=0.5)
        wait_digital_input(sig_num=1, desired_state=True)
        node.get_logger().info("Gripper closed")
        time.sleep(0.2)

    def gripper_measure():
        """
        노드 시작 시 그리퍼를 닫는 용도로 사용
        """
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        node.get_logger().info("Gripper measure (close)")
        rclpy.spin_once(node, timeout_sec=0.5)
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
        set_desired_force(fd=[0, 15, 0, 0, 0, 0], 
                          dir=[0, 1, 0, 0, 0, 0], 
                          mod=DR_FC_MOD_REL)
        node.get_logger().info("로봇이 +Y 방향으로 이동 중...")

        # Y축 힘이 2.5N 이상이면 충돌로 보고 멈춘다(예시)
        while not check_force_condition(DR_AXIS_Y, max=2.5):
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

        # 블록 개수 = 거리 / 블록 두께(16.75 mm)
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
        movej(Global_0)         # 최종적으로 초기 자세
        node.get_logger().info("[6] 측정 후 초기 자세 복귀 완료.")

    ######################################################################
    # 5. 메인 루프 (노드 실행 시 동작)
    ######################################################################
    try:
        # 시작 시 그리퍼를 열었다 닫아 초기화 (예시)
        gripper_release()
        movej(Global_0)

        # 그리퍼 닫힌 상태로 시작
        gripper_measure()
        node.get_logger().info("노드 시작: 그리퍼 닫힌 상태로 초기화")

        # 메인 동작: 블록 길이 측정
        node.get_logger().info("나무 블록의 길이를 측정하기 시작합니다.")
        measure_length_with_force_control()

        node.get_logger().info("측정을 모두 마쳤으므로 노드를 종료합니다.")
        rclpy.shutdown()

    except KeyboardInterrupt:
        node.get_logger().info("사용자 종료 요청(Ctrl+C). 노드를 종료합니다.")
        rclpy.shutdown()

######################################################################
# 보조 함수: 디지털 입력 가져오기 (실제 하드웨어에 맞게 수정 필요)
######################################################################
def get_digital_input(sig_num):
    """
    실제 환경에서는 하드웨어와 통신하여 디지털 입력을 읽어와야 합니다.
    여기서는 예시로 항상 True/False를 단순 반환하도록 구성합니다.
    """
    if sig_num == 1:
        return True
    return False
