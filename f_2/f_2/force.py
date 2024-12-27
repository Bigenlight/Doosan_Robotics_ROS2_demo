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
            set_velj,
            set_accj,
            set_velx,
            set_accx,
            task_compliance_ctrl,
            set_desired_force,
            check_force_condition,
            release_compliance_ctrl,
            set_singular_handling,
            gripper_grip,
            posj,
            posx,
            DR_AXIS_X,
            DR_FC_MOD_REL,
        )
    except ImportError as e:
        node.get_logger().error(f"두산 라이브러리 임포트 에러: {e}")
        return

    ######################################################################
    # 기본 셋팅
    ######################################################################
    # 초기 자세 (예시)
    Global_0 = posj(0.00, -0.01, 90.02, -0.01, 89.99, 0.01)

    # 속도/가속도 예시 세팅 (필요 시 조정)
    set_velj(50.0)
    set_accj(50.0)
    set_velx(50.0, 50.625)
    set_accx(50.0, 50.5)

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
    # 3. 보조 함수: 힘 제어를 이용한 길이 측정
    ######################################################################
    def measure_length_with_force_control():
        """
        힘 제어를 이용하여 오른쪽과 왼쪽 지점 사이의 길이를 측정합니다.
        """
        node.get_logger().info("길이 측정을 시작합니다.")

        # 초기 자세로 이동
        movej(Global_0)
        node.get_logger().info("초기 자세로 이동 완료.")
        time.sleep(1)

        ##################################################################
        # 오른쪽 지점에서 왼쪽으로 힘 제어하여 길이 측정
        ##################################################################
        # 오른쪽 위치로 이동
        movel(right_pose)
        node.get_logger().info("오른쪽 위치로 이동 완료.")

        # z축으로 5cm 위로 이동하여 충돌 방지
        delta_up = [0, 0, 50, 0, 0, 0]  # 50mm = 5cm
        approach_pose_right = trans(right_pose, delta_up)
        movel(approach_pose_right)
        node.get_logger().info("오른쪽 접근 자세로 이동 완료.")

        # 왼쪽 방향으로 힘 제어 설정
        task_compliance_ctrl(stx=[3000, 3000, 3000, 100, 100, 100])
        set_desired_force(fd=[-30, 0, 0, 0, 0, 0], dir=[1, 0, 0, 0, 0, 0], mod=DR_FC_MOD_REL)
        node.get_logger().info("힘 제어를 시작하여 왼쪽으로 이동 중...")

        # 힘 조건이 만족될 때까지 대기 (X축 기준)
        while not check_force_condition(DR_AXIS_X, max=5):
            time.sleep(0.1)

        # 현재 위치 측정
        current_pose_right_to_left = get_current_posx()
        pose_rl, status_rl = current_pose_right_to_left
        node.get_logger().info(f"오른쪽에서 왼쪽으로 힘 제어 후 현재 위치: {pose_rl}")

        # 힘 제어 해제
        release_compliance_ctrl()

        # 초기 위치와 현재 위치 간의 거리 계산 (X축 거리)
        distance_rl_mm = abs(pose_rl[0] - right_pose_list[0])
        distance_rl_cm = distance_rl_mm / 10.0  # mm to cm

        node.get_logger().info(f"오른쪽에서 왼쪽으로 이동한 거리: {distance_rl_cm:.2f} cm")

        ##################################################################
        # 왼쪽 지점에서 오른쪽으로 힘 제어하여 길이 측정
        ##################################################################
        # 왼쪽 위치로 이동
        movel(left_pose)
        node.get_logger().info("왼쪽 위치로 이동 완료.")

        # z축으로 5cm 위로 이동하여 충돌 방지
        approach_pose_left = trans(left_pose, delta_up)
        movel(approach_pose_left)
        node.get_logger().info("왼쪽 접근 자세로 이동 완료.")

        # 오른쪽 방향으로 힘 제어 설정
        task_compliance_ctrl(stx=[3000, 3000, 3000, 100, 100, 100])
        set_desired_force(fd=[30, 0, 0, 0, 0, 0], dir=[1, 0, 0, 0, 0, 0], mod=DR_FC_MOD_REL)
        node.get_logger().info("힘 제어를 시작하여 오른쪽으로 이동 중...")

        # 힘 조건이 만족될 때까지 대기 (X축 기준)
        while not check_force_condition(DR_AXIS_X, max=5):
            time.sleep(0.1)

        # 현재 위치 측정
        current_pose_left_to_right = get_current_posx()
        pose_lr, status_lr = current_pose_left_to_right
        node.get_logger().info(f"왼쪽에서 오른쪽으로 힘 제어 후 현재 위치: {pose_lr}")

        # 힘 제어 해제
        release_compliance_ctrl()

        # 초기 위치와 현재 위치 간의 거리 계산 (X축 거리)
        distance_lr_mm = abs(pose_lr[0] - left_pose_list[0])
        distance_lr_cm = distance_lr_mm / 10.0  # mm to cm

        node.get_logger().info(f"왼쪽에서 오른쪽으로 이동한 거리: {distance_lr_cm:.2f} cm")

        # 총 길이 계산 (양 방향 측정 합산)
        total_distance_cm = distance_rl_cm + distance_lr_cm
        block_count = round(total_distance_cm / BLOCK_THICKNESS_CM)  # 반올림하여 정수로 변환

        node.get_logger().info("=================================")
        node.get_logger().info("힘 제어를 통한 총 길이 측정 완료")
        node.get_logger().info(f" - 오른쪽에서 왼쪽으로 이동한 거리(cm): {distance_rl_cm:.2f} cm")
        node.get_logger().info(f" - 왼쪽에서 오른쪽으로 이동한 거리(cm): {distance_lr_cm:.2f} cm")
        node.get_logger().info(f" - 총 거리(cm): {total_distance_cm:.2f} cm")
        node.get_logger().info(f" - 나무 블록 개수(1cm 기준): {block_count} 개")
        node.get_logger().info("=================================")

        # 초기 자세로 복귀
        movel(approach_pose_right)  # 오른쪽 접근 자세로 이동
        movej(Global_0)
        node.get_logger().info("초기 자세로 복귀 완료.")

    ######################################################################
    # 4. 보조 함수: 높이 측정을 위한 함수 (기존 코드)
    ######################################################################
    def measure_heights(total_count, pr00, pr02, pr22, pr20, data):
        """
        각 pallet_index에 대해 높이를 측정하여 data 딕셔너리에 저장합니다.
        """
        for pallet_index in range(total_count):
            node.get_logger().info(f"[Right -> Left] Picking object at index: {pallet_index}")
            Pallet_Pose_r = get_pattern_point_3x3(pr00, pr02, pr22, pr20, pallet_index)

            # Approach from above
            delta = [0, 0, 50, 0, 0, 0]
            Pallet_Pose_r_up = trans(Pallet_Pose_r, delta)
            movel(Pallet_Pose_r_up)

            task_compliance_ctrl(stx=[3000, 3000, 3000, 100, 100, 100])
            set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            while not check_force_condition(DR_AXIS_Z, max=5):
                pass

            # Get current pose and extract height
            current_pose = get_current_posx()
            pose, status = current_pose
            height = pose[2]
            data[str(pallet_index)] = height

            # Release compliance control
            release_compliance_ctrl()
            node.get_logger().info(f"Completed measurement for index {pallet_index}")

            movel(Pallet_Pose_r_up)

    ######################################################################
    # 5. 보조 함수: 패턴 포인트 생성 (기존 코드)
    ######################################################################
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

    ######################################################################
    # 6. 보조 함수: 높이 데이터 검증 및 정렬 (기존 코드)
    ######################################################################
    def validate_heights(data):
        """
        높이가 71.61 이상 또는 20.0 이하인 경우 에러를 발생시킵니다.
        """
        for key, value in data.items():
            if value >= 71.61 or value <= 20.0:
                raise ValueError(f"Height at index {key} is {value}, which exceeds the allowed range.")

    def sort_heights(data):
        """
        높이를 기준으로 오름차순 정렬합니다.
        높이가 같은 경우, 키 값이 작은 순서대로 정렬합니다.
        """
        sorted_data = dict(sorted(data.items(), key=lambda item: (item[1], int(item[0]))))
        return sorted_data

    def sort_data_group(data):
        """
        높이를 그룹화하여 's', 'm', 'l'로 분류합니다.
        """
        new_data = {}
        for key, height in data.items():
            if 40 <= height <= 50:
                new_data[key] = 's'
            elif 50 < height <= 60:
                new_data[key] = 'm'
            else:
                new_data[key] = 'l'
        return new_data

    ######################################################################
    # 7. 메인 루프
    ######################################################################
    try:
        # 패턴 포인트 정의 (오른쪽과 왼쪽 팔레트의 코너 좌표)
        # 오른쪽 팔레트 코너
        pr00 = posx([500.0, -1.51,   21.61, 86.72, 179.94, 86.74])
        pr02 = posx([500.0, -103.51, 21.61, 86.72, 179.94, 86.74])
        pr22 = posx([398.0, -103.51, 21.61, 86.72, 179.94, 86.74])
        pr20 = posx([398.0, -1.51,   21.61, 86.72, 179.94, 86.74])

        # 왼쪽 팔레트 코너
        pl00 = posx([500.0, 147.0,  21.61, 6.55,  -179.64, 6.31])
        pl02 = posx([500.0, 45.0,   21.61, 6.55,  -179.64, 6.31])
        pl22 = posx([398.0, 45.0,   21.61, 6.55,  -179.64, 6.31])
        pl20 = posx([398.0, 147.0,  21.61, 6.55,  -179.64, 6.31])

        total_count = 9  # 3x3

        data = {}
        data_group = {}

        while rclpy.ok():
            node.get_logger().info("나무 블록 개수 측정을 시작합니다.")

            # 초기 자세로 이동
            movej(Global_0)
            node.get_logger().info("초기 자세로 이동 완료.")
            time.sleep(1)

            # 높이 측정
            gripper_measure()
            measure_heights(total_count, pr00, pr02, pr22, pr20, data)

            # 데이터 변환 및 그룹화
            node.get_logger().info(f"{data}\n")
            data_group = sort_data_group(data)
            node.get_logger().info(f"{data_group}\n")

            # 블록 개수 측정 (힘 제어를 통한 길이 측정)
            measure_length_with_force_control()

            # 필요 시 추가적인 로직을 여기에 삽입

            # 작업 완료 후 루프 반복 또는 종료
            node.get_logger().info("작업을 완료하고 루프를 다시 시작합니다.\n")
            rclpy.spin_once(node, timeout_sec=0.1)

        node.get_logger().info("노드를 종료합니다.")
        rclpy.shutdown()

    except KeyboardInterrupt:
        node.get_logger().info("사용자 종료 요청이 감지되어 노드를 종료합니다.")
        rclpy.shutdown()

if __name__ == "__main__":
    main()


