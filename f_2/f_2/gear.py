#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import DR_init
import time

# Doosan Robot model info
ROBOT_ID    = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id    = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON  = 1
OFF = 0

def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    node = rclpy.create_node("doosan_drl_new_task_example", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            # Robot motion APIs
            movej, movel, movesx, trans, amove_periodic,  # stop,
            # IO and force/torque APIs
            set_digital_output, get_digital_input,
            set_ref_coord, set_tool, set_tcp,
            task_compliance_ctrl, set_stiffnessx,
            set_desired_force, release_force,
            release_compliance_ctrl,
            check_force_condition,
            set_velj, set_accj, set_velx, set_accx,
            set_singular_handling,
            wait,
            get_current_posx,
            # wait_digital_input,
            DR_AXIS_Z, DR_BASE, DR_TOOL,
            DR_FC_MOD_REL, DR_MV_RA_DUPLICATE, DR_MV_APP_NONE, DR_MV_MOD_ABS,
            DR_AVOID, DR_QSTOP
        )
        from DR_common2 import (posj, posx)
    except ImportError as e:
        node.get_logger().error(f"Error importing Doosan libraries: {e}")
        return


    # As in the DRL snippet
    Global_0   = posj([0.00, -0.01, 90.02, -0.01, 89.99, 0.01])
    Global_app = posx([0.00, 0.00, 0.00, 0.00, 0.00, 0.00])
    Global_initial = posx([367.30, 6.25, 194.68, 101.75, 179.98, 101.76])

    # Example gear positions
    Global_Left_middle_gear_gripping_pose  = posx([500.40, 220.37, 38.73, 59.88, -179.91, 59.50])


    def detecting():
        """
        Example compliance + force-detecting function.
        Moves downward until a certain force or height triggers a stop,
        performs an amove_periodic, then stops if a certain Z threshold is reached.
        """
        # Set reference coordinate
        set_ref_coord(1)

        # Turn on compliance
        task_compliance_ctrl()
        set_stiffnessx([3000.0, 3000.0, 3000.0, 200.0, 200.0, 200.0], time=0.0)

        # Set desired force in Z direction (downward)
        set_desired_force([0.0, 0.0, 40.0, 0.0, 0.0, 0.0], [0, 0, 1, 0, 0, 0], time=0.0, mod=DR_FC_MOD_REL)

        # First while loop: check for high enough force condition (Z >= 10N)
        while rclpy.ok():
            time.sleep(0.05)
            node.get_logger().info(f"힘 여부: {check_force_condition(axis=DR_AXIS_Z, min=10, ref = DR_TOOL)}")
            if check_force_condition(axis=DR_AXIS_Z, min=20, ref = DR_TOOL):
                # Move periodic to sense or “shake”
                amove_periodic(
                    amp=[0.0, 0.0, 0.0, 0.0, 0.0, 8.0],
                    period=[0.0, 0.0, 0.0, 0.0, 0.0, 1.5],
                    atime=1, repeat=5, ref=1
                )
                break

        # Second while loop: check position or force to break
        while rclpy.ok():
            pos_chk = 0.0
            # Example of reading current Z (index 2)
            current_pose = get_current_posx(ref=DR_BASE)

            pose, status = current_pose  # (pose, status)
            height = pose[2]     # Z
            node.get_logger().info(f"현재 높이: {height:.2f}")

            # Example condition: if Z <= 50 => stop
            if height <= 50.0:
                node.get_logger().info("높이 도달, 중지")
                break

            time.sleep(0.05)

        # Release force control & compliance
        release_force(time=0.0)
        release_compliance_ctrl()

    def gripper_grip():
        """
        Closes the gripper. Waits for the digital input #1 to become ON,
        indicating the gripper is closed.
        """
        time_waiting_gripper = 0.5
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(time_waiting_gripper)
        wait_digital_input(1, ON)

    def gripper_release():
        """
        Opens the gripper. Waits for the digital input #2 to become ON,
        indicating the gripper is opened.
        """
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait(0.1)
        wait_digital_input(2, ON)

    def set_gripper():
        """
        Example setting of tool & TCP if needed.
        """
        set_tool("Tool Weight")
        set_tcp("GripperSA_v1")

    def wait_digital_input(sig_num, desired_state=True, period=0.2):
        while rclpy.ok():
            val = get_digital_input(sig_num)
            if val == desired_state:
                break
            time.sleep(period)
            print(f"Waiting for digital input #{sig_num} to be {desired_state}")

    ###########################################################################
    #                        Main (Pick-and-Place) Logic                      #
    ###########################################################################

    # Robot motion & singularity settings
    set_singular_handling(DR_AVOID)
    set_velj(30.0)
    set_accj(100.0)
    set_velx(100.0, 80.625)
    set_accx(100.0, 322.5)

    loop_count = 0
    while  loop_count < 1 and rclpy.ok():
        gripper_release()
        time.sleep(0.2)
        movej(Global_0, radius=0.0, ra=DR_MV_RA_DUPLICATE)

        # MoveSX #1: pick something
        movesx([
            posx([490.31, -22.75, 103.95, 2.28, -179.83, 2.62]),
            posx([494.28, -17.70, 41.06, 1.47, -179.73, 1.65])
        ], ref=0)
        gripper_grip()

        # MoveSX #2: place or transition
        movesx([
            posx([494.65, -17.74, 84.54, 12.46, -179.93, 12.65]),
            posx([443.52, 246.68, 79.61, 24.34, -179.67, 24.91]),
            posx([443.93, 248.05, 54.29, 25.13, -179.61, 25.60])
        ], ref=0)
        gripper_release()

        # MoveSX #3: pick again
        movesx([
            posx([445.16, -112.35, 51.57, 10.57, -179.71, 11.06]),
            posx([448.15, -111.76, 41.54, 7.37, -179.76, 7.74])
        ], ref=0)
        gripper_grip()

        # MoveSX #4
        movesx([
            posx([447.31, -111.53, 100.31, 2.31, -179.68, 2.70]),
            posx([547.12, 255.00, 95.95, 17.46, -179.89, 17.98]),
            posx([548.67, 256.32, 58.91, 23.58, 179.55, 24.03])
        ], ref=0)
        gripper_release()

        # MoveSX #5
        movesx([
            posx([550.19, -104.48, 55.54, 0.95, 179.71, 1.49]),
            posx([552.79, -105.13, 41.48, 34.63, 179.93, 35.17])
        ], ref=0)
        gripper_grip()

        # MoveSX #6
        movesx([
            posx([552.48, -104.83, 90.15, 2.11, 179.76, 2.63]),
            posx([501.35, 166.54, 87.81, 22.96, 179.76, 23.53]),
            posx([503.57, 163.02, 60.90, 26.28, -179.72, 26.95])
        ], ref=0)
        gripper_release()

        # MoveSX #7: final pick
        movesx([
            posx([498.88, -74.16, 60.37, 11.26, -179.74, 12.00]),
            posx([497.73, -77.82, 40.00, 176.76, 179.25, 177.89])
        ], ref=0)
        gripper_grip()

        # 연산
        delta = [0, 0, 70, 0, 0, 0]
        trans_result = trans(Global_Left_middle_gear_gripping_pose, delta)
        trans_result = trans_result.tolist()
        if isinstance(trans_result, list):
            Global_Left_middle_gear_gripping_above =trans_result
            node.get_logger().info(f"{Global_Left_middle_gear_gripping_above}")
        else:
            node.get_logger().error("trans() failed for gripping_above position.")
            rclpy.shutdown()
            return

        delta = [0, 0, 30, 0, 0, 0]
        trans_result = trans(Global_Left_middle_gear_gripping_pose, delta)
        trans_result = trans_result.tolist()
        if isinstance(trans_result, list):
            Global_Left_middle_gear_gripping_Set = trans_result
            node.get_logger().info(f"{Global_Left_middle_gear_gripping_Set}")
        else:
            node.get_logger().error("trans() failed for gripping_Set position.")
            rclpy.shutdown()
            return

        # MoveSX #8: approach middle gear
        movesx([
            posx([498.28, -77.71, 94.28, 177.56, 179.31, 178.68]),
            posx(Global_Left_middle_gear_gripping_above),
            posx(Global_Left_middle_gear_gripping_Set)
        ], ref=0)

        # Now call detecting()
        detecting()

        # Release gear
        gripper_release()

        # Return to home
        movej(Global_0, radius=0.0, ra=DR_MV_RA_DUPLICATE)

        # We only do 1 cycle here
        loop_count += 1
        break

    node.get_logger().info("Completed new DRL-like gear-moving task.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
