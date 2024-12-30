# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 100, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            movej,
            movel,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    JReady = [0, 0, 90, 0, 90, 0]
    offset = 102.0
    # pos1 = posx(500.0,    -1.51,         22.61, 86.72, 179.94, 86.74)
    # pos2 = posx(500.0,    -1.51 -offset, 22.61, 86.72, 179.94, 86.74)
    # pos3 = posx(500.0, 148.64,         22.61, 6.55,  -179.64, 6.31)

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    while rclpy.ok():

        # print("movej")
        # movej(JReady, vel=VELOCITY, acc=ACC)
        # print("movel")
        # movel(pos1, vel=VELOCITY, acc=ACC)
        # print("movel")
        # movel(pos2, vel=VELOCITY, acc=ACC)
        # print("movel")
        # movel(pos3, vel=VELOCITY, acc=ACC)
        # q3 = posj(64.492, 53.062, 86.909, -61.17, 111.002, 57.534) 
        # movej(posj(64.492, 53.062, 86.909, -61.17, 111.002, 57.534) , vel=20)

        Global_0 = posj(0.00, 0.0, 90.0, 0.0, 90.0, 0.0)
        movej(Global_0)

    rclpy.shutdown()
if __name__ == "__main__":
    main()
