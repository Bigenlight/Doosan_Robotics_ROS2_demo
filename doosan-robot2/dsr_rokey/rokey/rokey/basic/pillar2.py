#!/usr/bin/env python3
import rclpy
import DR_init
import time
ROBOT_ID    = "dsr01"
ROBOT_MODEL = "m0609"
ON = 1
OFF = 0
DR_init._dsr_id    = ROBOT_ID
DR_init._dsr_model = ROBOT_MODEL
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("doosan_drl_example", namespace=ROBOT_ID)
    DR_init._dsr_node = node
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
            initialize  # 초기화 함수 import
        )
        from DR_common2 import (posj, posx)
        # DSR_ROBOT2 초기화
        initialize()
    except ImportError as e:
        node.get_logger().error(f"Error importing Doosan libraries: {e}")
        return
    except ValueError as ve:
        node.get_logger().error(f"Initialization error: {ve}")
        return
    # -------------------------------
    # 보조 함수들
    # -------------------------------
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
        time.sleep(1)
    def gripper_release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        node.get_logger().info("Waiting 0.1s for release...")
        rclpy.spin_once(node, timeout_sec=0.1)
        wait_digital_input(sig_num=2, desired_state=True)
        node.get_logger().info("Gripper opened")
        time.sleep(1)
    # -------------------------------
    # 메인 동작
    # -------------------------------
    Global_0 = posj(0.00, -0.01, 90.02, -0.01, 89.99, 0.01)
    set_singular_handling()  # or pass a parameter if needed
    set_velj(80.0)
    set_accj(100.0)
    set_velx(250.0, 80.625)
    set_accx(1000.0, 322.5)
    # 오른쪽 팔레트 4개 모서리
    pr00 = posx([500.0,   -1.51,   21.61, 86.72, 179.94, 86.74])
    pr02 = posx([500.0, -103.51,   21.61, 86.72, 179.94, 86.74])
    pr22 = posx([398.0, -103.51,   21.61, 86.72, 179.94, 86.74])
    pr20 = posx([398.0,   -1.51,   21.61, 86.72, 179.94, 86.74])
    # 왼쪽 팔레트 4개 모서리
    pl00 = posx([500.0, 148.64,  21.61,  6.55, -179.64,  6.31])
    pl02 = posx([500.0,  46.64,  21.61,  6.55, -179.64,  6.31])
    pl22 = posx([398.0,  46.64,  21.61,  6.55, -179.64,  6.31])
    pl20 = posx([398.0, 148.64,  21.61,  6.55, -179.64,  6.31])
    total_count = 9  # 3x3
    while rclpy.ok():
        node.get_logger().info("Starting a pick-and-place cycle from Right Pallet -> Left Pallet...")
        gripper_release()
        movej(Global_0)
        # ---------------------------
        # (1) Right -> Left
        # ---------------------------
        for pallet_index in range(total_count):
            node.get_logger().info(f"[Right -> Left] Picking object at index: {pallet_index}")
            Pallet_Pose_r = get_pattern_point_3x3(pr00, pr02, pr22, pr20, pallet_index)
            delta = [0, 0, 70, 0, 0, 0]
            Pallet_Pose_r_up = trans(Pallet_Pose_r, delta)
            # 접근 & 피킹
            movel(Pallet_Pose_r_up)
            movel(Pallet_Pose_r)
            gripper_grip()
            # 왼쪽으로 이동
            Pallet_Pose_l = get_pattern_point_3x3(pl00, pl02, pl22, pl20, pallet_index)
            Pallet_Pose_l_up = trans(Pallet_Pose_l, delta)
            movel(Pallet_Pose_r_up)
            movel(Pallet_Pose_l_up)
            # 물건 배치 (조금 내려간 상태)
            alpha = [0, 0, 10, 0, 0, 0]
            Pallet_Pose_l_release = trans(Pallet_Pose_l, alpha)
            movel(Pallet_Pose_l_release)
            gripper_release()
            movel(Pallet_Pose_l_up)
        movej(Global_0)
        node.get_logger().info("Starting a pick-and-place cycle from Left Pallet -> Right Pallet...")
        # ---------------------------
        # (2) Left -> Right
        # “방향을 180도 뒤집어서” 놓기
        # ---------------------------
        for pallet_index in range(total_count):
            node.get_logger().info(f"[Left -> Right] Picking object at index: {pallet_index}")
            # (a) Left 팔레트에서 피킹 (원본 자세)
            Pallet_Pose_l = get_pattern_point_3x3(pl00, pl02, pl22, pl20, pallet_index)
            delta = [0, 0, 70, 0, 0, 0]
            Pallet_Pose_l_up = trans(Pallet_Pose_l, delta)
            movel(Pallet_Pose_l_up)
            movel(Pallet_Pose_l)
            gripper_grip()
            movel(Pallet_Pose_l_up)
            # (b) Right 팔레트로 이동 (하지만 놓을 땐 180도 뒤집어 놓기)
            Pallet_Pose_r = get_pattern_point_3x3(pr00, pr02, pr22, pr20, pallet_index)
            Pallet_Pose_r_up = trans(Pallet_Pose_r, delta)
            movel(Pallet_Pose_r_up)
            # Pallet_Pose_r에 180도 회전 추가 (예: c축 + 180)
            reversed_pose = list(Pallet_Pose_r)  # [x, y, z, a, b, c]
            reversed_pose[5] += 180.0             # c값에 180도 추가
            # 각도 값이 360도를 넘지 않도록 조정
            reversed_pose[5] = reversed_pose[5] % 360.0
            Pallet_Pose_r_reversed = posx(reversed_pose)
            # 실제 놓을 위치 (조금 내려간 상태)
            Pallet_Pose_r_release = trans(Pallet_Pose_r_reversed, alpha)
            movel(Pallet_Pose_r_release)
            gripper_release()
            # 다시 위로 복귀
            Pallet_Pose_r_up_reversed = trans(Pallet_Pose_r_reversed, delta)
            movel(Pallet_Pose_r_up_reversed)
        movej(Global_0)
        node.get_logger().info("One full cycle done! Looping again...\n")
        rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info("Shutting down node.")
    rclpy.shutdown()
if __name__ == "__main__":
    main()