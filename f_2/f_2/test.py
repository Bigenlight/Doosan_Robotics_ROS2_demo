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

# 테스트 데이터
valid_data_1 = {
    '1': 's',
    '2': 'm',
    '3': 'm',
    '4': 's',
    '5': 'n',
    '6': 'l',
    '7': 'l',
    '8': 'm',
    '9': 'l'  
    # s=2, m=3, l=3, n=1
}

if __name__ == '__main__':
    reorder_data_to_answer_until_done(valid_data_1)
