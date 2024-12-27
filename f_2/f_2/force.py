#!/usr/bin/env python3
import rclpy
import DR_init
import time
import math

ROBOT_ID    = "dsr01"
ROBOT_MODEL = "m0609"
ON = 1
OFF = 0

DR_init.__dsr__id    = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("doosan_drl_block_measure", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            movej,
            movel,
            trans,
            get_current_posx,
        )
        from DR_common2 import posj, posx
    except ImportError as e:
        node.get_logger().error(f"두산 라이브러리 임포트 에러: {e}")
        return

    ######################################################################
    # 기본 셋팅
    ######################################################################
    # 초기 자세 (예시)
    Global_0 = posj(0.00, -0.01, 90.02, -0.01, 89.99, 0.01)

    # 속도/가속도 예시 세팅 (필요 시 조정)
    from DSR_ROBOT2 import set_velj, set_accj, set_velx, set_accx
    set_velj(50.0)
    set_accj(50.0)
    set_velx(100.0, 30.625)
    set_accx(100.0, 50.5)

    ######################################################################
    # 1. 좌표 정의
    ######################################################################
    # 오른쪽 좌표 (질문에서 주어진 값)
    right_pose_list = [256.882, -254.63, 53.755, 30.124, -178.73, 29.768]
    # 왼쪽 좌표 (질문에서 주어진 값)
    left_pose_list  = [274.499, 261.32, 51.196, 145.692, -175.886, 139.237]

    # DSR 함수 형식 posx 로 포장
    right_pose = posx(right_pose_list)
    left_pose  = posx(left_pose_list)

    # 블록 두께 (1 cm)
    BLOCK_THICKNESS_CM = 1.0

    ######################################################################
    # 2. 보조 함수: 유클리드 거리 계산
    ######################################################################
    def calc_distance_3d(p1, p2):
        """
        p1, p2는 [x, y, z, rx, ry, rz] 형태의 float 리스트
        rx, ry, rz(오리엔테이션)는 거리 계산과 무관하므로 x,y,z만 사용
        """
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dz = p2[2] - p1[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    ######################################################################
    # (방법1) 실제 로봇 이동을 통해 측정
    #         - 로봇을 오른쪽 좌표로 이동 -> get_current_posx() -> 좌표 확인
    #         - 로봇을 왼쪽 좌표로 이동 -> get_current_posx() -> 좌표 확인
    #         - 두 좌표 차이로 블록 개수 계산
    ######################################################################
    def measure_by_robot_movement():
        # 초기 자세로 이동
        movej(Global_0)
        node.get_logger().info("초기 자세로 이동 완료.")

        # 오른쪽 위치로 이동
        movel(right_pose)
        # 실제 좌표 측정
        current_posx_right, _ = get_current_posx()
        node.get_logger().info(f"오른쪽 지점 측정 좌표: {current_posx_right}")

        # 왼쪽 위치로 이동
        movel(left_pose)
        # 실제 좌표 측정
        current_posx_left, _ = get_current_posx()
        node.get_logger().info(f"왼쪽 지점 측정 좌표: {current_posx_left}")

        # 다시 초기 자세로 복귀
        movej(Global_0)

        # 3D 거리 계산
        distance_mm = calc_distance_3d(current_posx_right, current_posx_left)
        # mm -> cm 변환
        distance_cm = distance_mm / 10.0

        # 블록 개수 계산 (1 cm 당 1개)
        block_count = distance_cm / BLOCK_THICKNESS_CM

        node.get_logger().info("=================================")
        node.get_logger().info(f" (방법1) 실제 로봇 이동으로 측정 ")
        node.get_logger().info(f"  - 두 지점 사이 거리(cm): {distance_cm:.3f}")
        node.get_logger().info(f"  - 나무 블록 개수(1cm 기준): {block_count:.1f} 개 (실수일 경우 반올림 가능)")
        node.get_logger().info("=================================")

    ######################################################################
    # (방법2) 미리 주어진 좌표 값만으로 계산
    #         - 로봇 이동 없이, 두 좌표의 차이만으로 블록 개수 계산
    ######################################################################
    def measure_by_calculation():
        # 두 좌표 posx 객체에서 내부 리스트를 꺼내기
        p_right = right_pose.get_data()  # [x, y, z, rx, ry, rz]
        p_left  = left_pose.get_data()

        # 3D 거리 계산 (mm 단위)
        distance_mm = calc_distance_3d(p_right, p_left)
        distance_cm = distance_mm / 10.0

        block_count = distance_cm / BLOCK_THICKNESS_CM

        node.get_logger().info("=================================")
        node.get_logger().info(f" (방법2) 좌표 계산만으로 측정 ")
        node.get_logger().info(f"  - 두 지점 사이 거리(cm): {distance_cm:.3f}")
        node.get_logger().info(f"  - 나무 블록 개수(1cm 기준): {block_count:.1f} 개 (실수일 경우 반올림 가능)")
        node.get_logger().info("=================================")

    ######################################################################
    # 메인 루프 (예시)
    ######################################################################
    try:
        while rclpy.ok():
            # (방법1) 실제 로봇 이동 후 측정
            measure_by_robot_movement()

            # (방법2) 계산 기반 측정
            measure_by_calculation()

            # 필요하다면 주기적으로 측정할 수 있음
            time.sleep(5.0)

            # 반복이 필요 없다면 break 처리
            # break

        node.get_logger().info("작업을 종료합니다.")
        rclpy.shutdown()

    except KeyboardInterrupt:
        node.get_logger().info("사용자 종료 요청이 감지되어 노드를 종료합니다.")
        rclpy.shutdown()

if __name__ == "__main__":
    main()
