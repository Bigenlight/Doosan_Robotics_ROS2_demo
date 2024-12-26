#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
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
            movej,
            movel,
            set_singular_handling,
            set_velj,
            set_accj,
            set_velx,
            set_accx,
            trans,
        )
        from DR_common2 import (posj, posx)

    except ImportError as e:
        node.get_logger().error(f"Error importing Doosan libraries: {e}")
        return

    # For a 3x3 pattern => 9 positions
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
        time.sleep(0.5)

    def gripper_release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        node.get_logger().info("Waiting 0.1s for release...")
        rclpy.spin_once(node, timeout_sec=0.1)
        wait_digital_input(sig_num=2, desired_state=True)
        node.get_logger().info("Gripper opened")
        time.sleep(0.5)

    def get_heights():
        """
        각 패렛의 높이를 감지하여 딕셔너리에 저장합니다.
        실제 센서 데이터를 사용하는 경우, 이 함수를 수정하여 센서 데이터를 읽어오세요.
        여기서는 테스트를 위해 임의의 데이터를 반환합니다.
        """
        # 예시 데이터 (패렛 인덱스: 높이)
        # 실제 구현에서는 센서 데이터를 읽어와야 합니다.
        data = {
            '0': 10.42,
            '1': 15.33,
            '2': 13.22,
            '3': 20.00,
            '4': 19.50,
            '5': 18.75,
            '6': 10.44,
            '7': 14.10,
            '8': 10.50
        }
        return data

    def validate_heights(data):
        """
        높이가 21.61 이상인 경우 에러를 발생시킵니다.
        """
        for key, value in data.items():
            if value >= 21.61:
                raise ValueError(f"Height at index {key} is {value}, which exceeds the maximum allowed value of 21.61.")

    def sort_heights(data):
        """
        높이를 기준으로 오름차순 정렬합니다.
        높이가 같은 경우, 키 값이 작은 순서대로 정렬합니다.
        """
        sorted_data = dict(sorted(data.items(), key=lambda item: (item[1], int(item[0]))))
        return sorted_data

    def pick_and_place_sorted(sorted_data):
        """
        정렬된 데이터를 기반으로 pick-and-place 작업을 수행합니다.
        Left Pallet에 배치할 때 pallet_index를 0부터 순차적으로 증가시킵니다.
        """
        new_pallet_index = 0  # Left Pallet의 새로운 인덱스

        for key, height in sorted_data.items():
            node.get_logger().info(f"Processing pallet_index {key} with height {height}")

            # 원래 pallet_index는 key, 새로운 인덱스는 new_pallet_index
            # Left Pallet의 위치를 새로운 인덱스로 설정
            Pallet_Pose_l = get_pattern_point_3x3(pl00, pl02, pl22, pl20, new_pallet_index)
            Pallet_Pose_l_up = trans(Pallet_Pose_l, [0, 0, 70, 0, 0, 0])

            # Approach from above
            movel(Pallet_Pose_l_up)
            movel(Pallet_Pose_l)

            # Gripper을 사용하여 물체 집기
            gripper_grip()

            # 이동 후 그리퍼 열기
            movel(Pallet_Pose_l_up)
            gripper_release()

            new_pallet_index += 1  # 다음 Left Pallet 위치로 이동

    Global_0 = posj(0.00, -0.01, 90.02, -0.01, 89.99, 0.01)

    set_singular_handling()  # or pass a parameter if needed
    set_velj(50.0)
    set_accj(50.0)
    set_velx(70.0, 30.625)
    set_accx(100.0, 50.5)

    # Right pallet corners
    pr00 = posx([500.0, -1.51,   21.61, 86.72, 179.94, 86.74])
    pr02 = posx([500.0, -103.51, 21.61, 86.72, 179.94, 86.74])
    pr22 = posx([398.0, -103.51, 21.61, 86.72, 179.94, 86.74])
    pr20 = posx([398.0, -1.51,   21.61, 86.72, 179.94, 86.74])

    # Left pallet corners
    pl00 = posx([500.0, 147.0,  21.61, 6.55,  -179.64, 6.31])
    pl02 = posx([500.0, 45.0,   21.61, 6.55,  -179.64, 6.31])
    pl22 = posx([398.0, 45.0,   21.61, 6.55,  -179.64, 6.31])
    pl20 = posx([398.0, 147.0,  21.61, 6.55,  -179.64, 6.31])

    switch_dir_right =  posx([637.261, 21.246, 102.717, 91.62, 90.867, -89.742])
    switch_dir_left =  posx([571.379, -11.415, 35.363, 90.057, -94.83, -93.092])
    switch_dir_left_j1 = posj([19.256, 67.053, 4.966, 102.548, -119.048, -203.422])
    switch_dir_left_j2 = posj([29.483, 75.225, 39.167, 101.148, -116.943, -152.729])
    switch_dir_left_j3 = posj([8.789, 35.952, 39.319, 64.225, 75.029, -209.713])

    total_count = 9  # 3x3

    while rclpy.ok():
        node.get_logger().info("Starting a pick-and-place cycle from Right Pallet -> Left Pallet...")

        # 높이 감지
        try:
            data = get_heights()
            validate_heights(data)
            sorted_data = sort_heights(data)
            node.get_logger().info(f"Sorted data: {sorted_data}")
        except ValueError as e:
            node.get_logger().error(f"Height validation error: {e}")
            break  # 에러 발생 시 루프 종료

        # Pick and Place based on sorted heights
        pick_and_place_sorted(sorted_data)

        node.get_logger().info("Pick-and-place from Right Pallet -> Left Pallet done!")

    node.get_logger().info("Shutting down node.")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
