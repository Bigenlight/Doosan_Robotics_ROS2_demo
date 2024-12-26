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

from collections import Counter
import time

def create_ans_list_with_n(data):
    """
    주어진 data 딕셔너리의 값들을 기반으로 정답 배열(ans)을 생성합니다.
    s, m, l의 개수를 세고, 's', 'm', 'l' 순서로 쌓은 뒤, 일정 간격(n_interval)마다 'n'을 삽입합니다.
    """
    counter = Counter(data.values())
    ans = []
    order = ['s', 'm', 'l']
    
    for item in order:
        ans.extend([item] * counter.get(item, 0))

    n_interval = 5
    current_index = n_interval - 1  
    while current_index < len(ans):
        ans.insert(current_index, 'n')
        current_index += n_interval + 1  
    
    return ans

# 데이터 switch 함수 수정
def switch_pillar_data(data_dict, key1, key2):
    """
    딕셔너리에서 두 키의 값을 교환하고, 교환된 데이터를 출력합니다.
    
    Parameters:
    - data_dict: 값을 스위치할 딕셔너리
    - key1: 첫 번째 키
    - key2: 두 번째 키
    """
    data_dict[key1], data_dict[key2] = (
        data_dict[key2],
        data_dict[key1],
    )
    print(f"Switched keys '{key1}' and '{key2}': {data_dict}")

# 실제 물리적으로 switch하는 함수
def switch_pillar(key1, key2):
    """
    실제 물리적 스위치를 구현하는 함수입니다.
    현재는 구현되지 않았으며, 필요한 로직을 추가해야 합니다.
    
    Parameters:
    - key1: 첫 번째 키
    - key2: 두 번째 키
    """
    # 실제 물리적 스위치 로직을 여기에 구현하세요.
    pass  # 실제 로직을 구현하세요

def reorder_data_to_answer_until_done(data, max_switches=100):
    """
    data 딕셔너리를 create_ans_list_with_n(data) 결과와 동일하게 만들기 위해
    앞에서부터 검사 -> 스위치 -> 다시 검사... 를 전체가 맞을 때까지 반복합니다.
    각 스위치가 발생할 때마다 현재 data의 상태를 출력합니다.
    
    Parameters:
    - data: 값을 재정렬할 딕셔너리
    - max_switches: 최대 스위치 횟수 (무한 루프 방지용)
    """
    ans = create_ans_list_with_n(data)
    keys = sorted(data.keys(), key=lambda x: int(x))  # data의 키를 숫자 순으로 정렬
    length = min(len(keys), len(ans))  # 길이 맞추기
    
    print("초기 데이터:", data)
    print("정답 배열(ans):", ans)
    
    switch_count = 0  # 스위치 횟수 카운트
    
    for i in range(length):
        while data[keys[i]] != ans[i]:
            if switch_count >= max_switches:
                print("\n[Error] 최대 스위치 횟수에 도달했습니다. 재정렬을 중단합니다.")
                return data
            
            current_val = data[keys[i]]
            target_val = ans[i]
            
            if current_val != 'n':
                # 'n'을 찾아서 스위치
                try:
                    n_key = next(key for key, value in data.items() if value == 'n')
                except StopIteration:
                    print("\n[Error] 'n'을 찾지 못했습니다. 스위치하지 않습니다.")
                    break  # 'n'이 없으면 더 이상 스위치할 수 없으므로 다음 인덱스로 넘어감
                
                print(f"\n[Switch] '{keys[i]}' (index {i}) with 'n' at '{n_key}' (index {keys.index(n_key)})")
                switch_pillar(keys[i], n_key)  # 물리 스위치
                switch_pillar_data(data, keys[i], n_key)  # 딕셔너리 스위치
                switch_count += 1
                time.sleep(1)  # 1초 딜레이 추가
                
            else:
                # 현재 값이 'n'인 경우, ans[i]와 같은 값을 찾음
                # ans[i]는 'n'이 아니므로, ans[i]가 있는 index를 찾음
                desired_val = target_val
                desired_key = None
                for j in range(i+1, length):
                    if data[keys[j]] == desired_val:
                        desired_key = keys[j]
                        break
                
                if desired_key is not None:
                    # 'n'과 desired_key를 스위치
                    try:
                        n_key = next(key for key, value in data.items() if value == 'n')
                    except StopIteration:
                        print("\n[Error] 'n'을 찾지 못했습니다. 스위치하지 않습니다.")
                        break  # 'n'이 없으면 더 이상 스위치할 수 없으므로 다음 인덱스로 넘어감
                    
                    print(f"\n[Switch] '{keys[i]}' (index {i}) with '{desired_key}' (index {keys.index(desired_key)})")
                    switch_pillar(keys[i], desired_key)  # 물리 스위치
                    switch_pillar_data(data, keys[i], desired_key)  # 딕셔너리 스위치
                    switch_count += 1
                    time.sleep(1)  # 1초 딜레이 추가
                else:
                    # 'n'과 교환할 target_val이 없으면, 5번째 요소와 스위치
                    desired_index = 4
                    if desired_index >= length:
                        desired_index = length -1
                        print(f"\n[Info] 원하는 인덱스 이후에 '{desired_val}'을 찾지 못했습니다. 마지막 인덱스({desired_index})와 스위치합니다.")
                    else:
                        print(f"\n[Info] 원하는 인덱스 이후에 '{desired_val}'을 찾지 못했습니다. 5번째 인덱스({desired_index})와 스위치합니다.")
                    
                    # 'n'을 찾음
                    try:
                        n_key = next(key for key, value in data.items() if value == 'n')
                    except StopIteration:
                        print("\n[Error] 'n'을 찾지 못했습니다. 스위치하지 않습니다.")
                        break  # 'n'이 없으면 더 이상 스위치할 수 없으므로 다음 인덱스로 넘어감
                    
                    switch_pillar(keys[desired_index], n_key)  # 물리 스위치
                    switch_pillar_data(data, keys[desired_index], n_key)  # 딕셔너리 스위치
                    switch_count += 1
                    time.sleep(1)  # 1초 딜레이 추가
                
    # 최종 확인
    all_match = all(data[keys[i]] == ans[i] for i in range(length))
    if all_match:
        print("\n모든 데이터가 정답 배열과 일치합니다. 재정렬을 종료합니다.")
    else:
        print("\n[Error] 일부 데이터가 정답 배열과 일치하지 않습니다.")

    return data


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

        gripper_release()
        reorder_data_to_answer_until_done(data_group)

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
