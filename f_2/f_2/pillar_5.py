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
            # movesx,  # We won't use movesx
            set_singular_handling,
            set_velj,
            set_accj,
            set_velx,
            set_accx,
            trans,
            DR_AXIS_Z,
            DR_FC_MOD_REL,
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

    def validate_heights(data):
        """
        높이가 21.61 이상인 경우 에러를 발생시킵니다.
        """
        for key, value in data.items():
            if value >= 71.61 or value <= 20.0:
                raise ValueError(f"Height at index {key} is {value}, which exceeds the maximum allowed value of 21.61.")

    def sort_heights(data):
        """
        높이를 기준으로 오름차순 정렬합니다.
        높이가 같은 경우, 키 값이 작은 순서대로 정렬합니다.
        """
        sorted_data = dict(sorted(data.items(), key=lambda item: (item[1], int(item[0]))))
        return sorted_data
    

    def sort_data_group(data):
        new_data = {}
        for key, height in data.items():
            if height >= 40 and height <= 50:
                new_data[key] = 's'
            elif height >= 50 and height <= 60:
                new_data[key] = 'm'
            else:
                new_data[key] = 'l'
        return new_data
                
    def switch_pillar(current_index, switching_index):
        current_index = int(current_index)
        switching_index = int(switching_index)
        gripper_release()
        Pallet_Pose_r = get_pattern_point_3x3(pr00, pr02, pr22, pr20, current_index)
        delta = [0, 0, 70, 0, 0, 0]
        Pallet_Pose_r_up = trans(Pallet_Pose_r, delta)
        # 접근 & 피킹
        movel(Pallet_Pose_r_up)
        movel(Pallet_Pose_r)
        gripper_grip()
        movel(Pallet_Pose_r_up)
        # 왼쪽으로 이동
        Pallet_Pose_l = get_pattern_point_3x3(pr00, pr02, pr22, pr20, switching_index)
        Pallet_Pose_l_up = trans(Pallet_Pose_l, delta)
        movel(Pallet_Pose_r_up)
        movel(Pallet_Pose_l_up)
        # 물건 배치 (조금 내려간 상태)
        alpha = [0, 0, 10, 0, 0, 0]
        Pallet_Pose_l_release = trans(Pallet_Pose_l, alpha)
        movel(Pallet_Pose_l_release)
        gripper_release()
        movel(Pallet_Pose_l_up)

    Global_0 = posj(0.00, -0.01, 90.02, -0.01, 89.99, 0.01)

    set_singular_handling()  # or pass a parameter if needed
    set_velj(50.0)
    set_accj(50.0)
    set_velx(100.0, 30.625)
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


    total_count = 9  # 3x3

    data = {}
    data_group = {}



    while rclpy.ok():
        node.get_logger().info("Starting a pick-and-place cycle from Right Pallet -> Left Pallet...")
        gripper_release()
        movej(Global_0)
        time.sleep(1)

        gripper_measure()
        ### 높이 측정
        for pallet_index in range(total_count):
            node.get_logger().info(f"[Right -> Left] Picking object at index: {pallet_index}")
            Pallet_Pose_r = get_pattern_point_3x3(pr00, pr02, pr22, pr20, pallet_index)

            # Approach from above
            delta = [0, 0, 50, 0, 0, 0]
            Pallet_Pose_r_up = trans(Pallet_Pose_r, delta)
            movel(Pallet_Pose_r_up)

            task_compliance_ctrl(stx=[3000, 3000, 3000, 100, 100, 100])
            set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
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

        # data 변환
        node.get_logger().info(f"{data}\n")
        data_group = sort_data_group(data)
        node.get_logger().info(f"{data_group}\n")

        ######## 5번 빼기
        gripper_release()
        Pallet_Pose_r = get_pattern_point_3x3(pr00, pr02, pr22, pr20, 4)
        delta = [0, 0, 70, 0, 0, 0]
        Pallet_Pose_r_up = trans(Pallet_Pose_r, delta)
        # 접근 & 피킹
        movel(Pallet_Pose_r_up)
        movel(Pallet_Pose_r)
        gripper_grip()
        movel(Pallet_Pose_r_up)
        # 왼쪽으로 이동
        Pallet_Pose_l = get_pattern_point_3x3(pl00, pl02, pl22, pl20, 4)
        Pallet_Pose_l_up = trans(Pallet_Pose_l, delta)
        movel(Pallet_Pose_r_up)
        movel(Pallet_Pose_l_up)
        # 물건 배치 (조금 내려간 상태)
        alpha = [0, 0, 10, 0, 0, 0]
        Pallet_Pose_l_release = trans(Pallet_Pose_l, alpha)
        movel(Pallet_Pose_l_release)
        gripper_release()
        movel(Pallet_Pose_l_up)
        #####################

        ############# 정렬 로직 시작
        #result = []
        if data_group['4'] == 's':
            result = ['s', 's', 'm', 'm', 'n', 'm', 'l', 'l', 'l']
        elif data_group['4'] == 'm':
            result = ['s', 's', 's', 'm', 'n', 'm', 'l', 'l', 'l']
        elif data_group['4'] == 'l':
            result = ['s', 's', 's', 'm', 'n', 'm', 'm', 'l', 'l']
        data_group['4'] = 'n'

        current_index = 0
        new_pallet_index = 0  # Left Pallet의 새로운 인덱스
        
        gripper_release()

        for key, group in data_group.items():
            i = 0
            node.get_logger().info(f"Processing pallet_index {key} with height {group}")
            if current_index == 4:
                pass
            elif data_group[key] == result[current_index]:
                pass
            elif data_group[key] == 'n':
                target_list = [key for key, value in data_group.items() if value == result[current_index]]
                while target_list[-1 - i] == result[int(target_list[-1 - i])]:
                    i += 1

                switch_pillar(target_list[-1 - i], str(current_index))
                x = data_group[key]
                data_group[key] = data_group[str(target_list[-1 - i])]
                data_group[str(target_list[-1 - i])] = x
                
            else :
                target_list = [key for key, value in data_group.items() if value == 'n']
                switch_pillar(str(current_index), target_list[-1])
                x = data_group[key]
                data_group[key] = data_group[str(target_list[-1])]
                data_group[str(target_list[-1])] = x
                
                target_list = [key for key, value in data_group.items() if value == result[current_index]]
                while target_list[-1 - i] == result[int(target_list[-1 - i])]:
                    i += 1

                # if current_index >= 6 and int(target_list[-1]) > current_index and int(target_list[-1]) <= 8:
                #     switch_pillar(target_list[-1], str(current_index))
                #     x = data_group[key]
                #     data_group[key] = data_group[str(target_list[-1 + i])]
                #     data_group[str(target_list[-1])] = x
                #     current_index  += 1
                #     pass
                # if current_index >= 3 and current_index <= 5 and int(target_list[-1]) > current_index and int(target_list[-1]) <= 5:
                #     i = 1
                #     switch_pillar(target_list[-1], str(current_index))
                #     x = data_group[key]
                #     data_group[key] = data_group[str(target_list[-1 + i])]
                #     data_group[str(target_list[-1])] = x
                #     current_index  += 1
                #     pass
                switch_pillar(target_list[-1 - i], str(current_index))
                x = data_group[key]
                data_group[key] = data_group[str(target_list[-1 - i])]
                data_group[str(target_list[-1 - i])] = x

            current_index  += 1  # 다음 Left Pallet 위치로 이동

        if  data_group['4'] != 'n':
            target_list = [key for key, value in data_group.items() if value == 'n']
            switch_pillar('4', target_list[0])
            x = data_group[key]
            data_group[key] = data_group[str(target_list[0])]
            data_group[str(target_list[0])] = x
        ################ 정렬 로직 끝

        ######## 5번 복구
        gripper_release()
        Pallet_Pose_l = get_pattern_point_3x3(pl00, pl02, pl22, pl20, 4)
        delta = [0, 0, 70, 0, 0, 0]
        Pallet_Pose_l_up = trans(Pallet_Pose_l, delta)
        # 접근 & 피킹
        movel(Pallet_Pose_l_up)
        movel(Pallet_Pose_l)
        gripper_grip()
        movel(Pallet_Pose_l_up)
        # 왼쪽으로 이동
        Pallet_Pose_r = get_pattern_point_3x3(pr00, pr02, pr22, pr20, 4)
        Pallet_Pose_r_up = trans(Pallet_Pose_r, delta)
        movel(Pallet_Pose_r_up)
        # 물건 배치 (조금 내려간 상태)
        alpha = [0, 0, 10, 0, 0, 0]
        Pallet_Pose_r_release = trans(Pallet_Pose_r, alpha)
        movel(Pallet_Pose_r_release)
        gripper_release()
        movel(Pallet_Pose_r_up)
        #####################

        gripper_release()
        
        movej(Global_0)
        node.get_logger().info("One full cycle done! Looping again...\n")
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info("Shutting down node.")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
