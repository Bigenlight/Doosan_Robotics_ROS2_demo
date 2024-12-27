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

    # 좌표 정의
    # --------------------------------------------------
    # 오른쪽 '위' 좌표 (충돌 방지용 오버헤드)
    right0_pose_list = [256.882, -254.63, 123.755, 30.124, -178.73, 29.768]
    # 왼쪽 '위' 좌표 (충돌 방지용 오버헤드)
    left0_pose_list  = [274.499,  261.32, 121.196, 145.692, -175.886, 139.237]
    # 오른쪽 (실제 측정 지점)
    right_pose_list = [256.882, -254.63, 53.755, 30.124, -178.73, 29.768]
    # 왼쪽 (실제 측정 지점)
    left_pose_list  = [274.499,  261.32, 51.196, 145.692, -175.886, 139.237]
    # --------------------------------------------------

    # Doosan posx 형태로 감싸기
    right0_pose = posx(right0_pose_list)
    left0_pose  = posx(left0_pose_list)
    right_pose  = posx(right_pose_list)
    left_pose   = posx(left_pose_list)

    # 나무 블록 두께(1 cm)
    BLOCK_THICKNESS_CM = 1.6

    ######################################################################
    # 3. 디지털 IO 제어 및 그리퍼 함수
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
        node.get_logger().info("Waiting 0.1s for release...")
        rclpy.spin_once(node, timeout_sec=0.1)
        wait_digital_input(sig_num=2, desired_state=True)
        node.get_logger().info("Gripper opened")
        time.sleep(0.2)

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

    def gripper_measure():
        """
        코드 예시에 따라, 시작 시 그리퍼를 닫는 용도로 사용
        """
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        node.get_logger().info("Measure (close gripper)...")
        rclpy.spin_once(node, timeout_sec=0.5)
        node.get_logger().info("Gripper closed")
        time.sleep(0.3)

    ######################################################################
    # 4. 힘 제어를 이용한 길이 측정 (오른쪽 → +Y, 왼쪽 → -Y)
    ######################################################################
    def measure_length_with_force_control():
        """
        1) 오른쪽(실제 측정 지점)에서 +Y로 힘 제어하여 거리 측정
        2) 왼쪽(실제 측정 지점)에서 -Y로 힘 제어하여 거리 측정
        3) 두 값을 합산해 총 길이 산출 및 블록 개수 계산
        """

        node.get_logger().info("길이 측정을 시작합니다.")

        ##################################################################
        # [A] 초기 자세 → 오른쪽 위 → 오른쪽
        ##################################################################
        movej(Global_0)
        node.get_logger().info("초기 자세(Global_0) 이동 완료.")
        time.sleep(0.5)

        movel(right0_pose)
        node.get_logger().info("오른쪽 위 좌표(right0_pose)로 이동 완료.")

        movel(right_pose)
        node.get_logger().info("오른쪽 실제 측정 좌표(right_pose)로 이동 완료.")

        ##################################################################
        # [B] +Y 힘 제어 (오른쪽 → 왼쪽 방향 측정)
        ##################################################################
        node.get_logger().info("오른쪽(+Y) 힘 제어를 시작합니다.")
        task_compliance_ctrl(stx=[3000, 3000, 3000, 100, 100, 100])
        set_desired_force(fd=[0, 1, 0, 0, 0, 0], dir=[0, 1, 0, 0, 0, 0], mod=DR_FC_MOD_REL)

        while not check_force_condition(DR_AXIS_Y, max=1.5):
            time.sleep(0.1)

        current_pose_r2l = get_current_posx()
        pose_r2l, _ = current_pose_r2l
        node.get_logger().info(f"오른쪽 → 왼쪽(+Y) 힘 제어 후 위치: {pose_r2l}")

        release_compliance_ctrl()

        distance_r2l_mm = abs(pose_r2l[1] - right_pose_list[1])
        distance_r2l_cm = distance_r2l_mm / 10.0
        node.get_logger().info(f"[+Y 측정] 이동 거리: {distance_r2l_cm:.2f} cm")

        ##################################################################
        # [C] 오른쪽 → 오른쪽 위 → 왼쪽 위 → 왼쪽
        ##################################################################
        # 힘 제어 후, 다시 오른쪽 좌표로 복귀(잔여 이동 최소화)
        movel(right_pose)
        node.get_logger().info("오른쪽 좌표로 복귀 완료.")

        movel(right0_pose)
        node.get_logger().info("오른쪽 위 좌표(right0_pose)로 이동 완료.")

        movel(left0_pose)
        node.get_logger().info("왼쪽 위 좌표(left0_pose)로 이동 완료.")

        movel(left_pose)
        node.get_logger().info("왼쪽 실제 측정 좌표(left_pose)로 이동 완료.")

        ##################################################################
        # [D] -Y 힘 제어 (왼쪽 → 오른쪽 방향 측정)
        ##################################################################
        node.get_logger().info("왼쪽(-Y) 힘 제어를 시작합니다.")
        task_compliance_ctrl(stx=[3000, 3000, 3000, 100, 100, 100])
        set_desired_force(fd=[0, -1, 0, 0, 0, 0], dir=[0, 1, 0, 0, 0, 0], mod=DR_FC_MOD_REL)

        while not check_force_condition(DR_AXIS_Y, max=1.5):
            time.sleep(0.1)

        current_pose_l2r = get_current_posx()
        pose_l2r, _ = current_pose_l2r
        node.get_logger().info(f"왼쪽 → 오른쪽(-Y) 힘 제어 후 위치: {pose_l2r}")

        release_compliance_ctrl()

        distance_l2r_mm = abs(pose_l2r[1] - left_pose_list[1])
        distance_l2r_cm = distance_l2r_mm / 10.0
        node.get_logger().info(f"[-Y 측정] 이동 거리: {distance_l2r_cm:.2f} cm")

        ##################################################################
        # [E] 왼쪽 → 왼쪽 위 → 오른쪽 위 → 초기 자세
        ##################################################################
        # 힘 제어 후, 다시 왼쪽 좌표로 복귀
        movel(left_pose)
        node.get_logger().info("왼쪽 좌표로 복귀 완료.")

        movel(left0_pose)
        node.get_logger().info("왼쪽 위 좌표(left0_pose)로 이동 완료.")

        movel(right0_pose)
        node.get_logger().info("오른쪽 위 좌표(right0_pose)로 이동 완료.")

        movej(Global_0)
        node.get_logger().info("초기 자세(Global_0) 복귀 완료.")

        ##################################################################
        # [F] 최종 계산: 블록 개수
        ##################################################################
        total_distance_cm = distance_r2l_cm + distance_l2r_cm
        block_count = round(total_distance_cm / BLOCK_THICKNESS_CM)

        node.get_logger().info("=================================")
        node.get_logger().info("힘 제어를 통한 총 길이 측정 완료")
        node.get_logger().info(f" - 오른쪽(+Y) 측정 거리: {distance_r2l_cm:.2f} cm")
        node.get_logger().info(f" - 왼쪽(-Y) 측정 거리:  {distance_l2r_cm:.2f} cm")
        node.get_logger().info(f" - 총 거리:             {total_distance_cm:.2f} cm")
        node.get_logger().info(f" - 나무 블록 개수:      {block_count} 개 (두께 1cm 기준)")
        node.get_logger().info("=================================")

    ######################################################################
    # 5. 노드 실행 시 동작 (메인 로직)
    ######################################################################
    try:

        gripper_release()
        movej(Global_0)
        # 노드 실행 직후, 그리퍼를 닫은 상태로 시작
        gripper_measure()
        node.get_logger().info("노드 시작 시 그리퍼를 닫은 상태로 초기화했습니다.")

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
    if sig_num == 1:
        return True
    return False
