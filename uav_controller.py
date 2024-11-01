import threading
from pymavlink import mavutil
import time


class UAVController:
    def __init__(self, connection_string):
        self.connection = mavutil.mavlink_connection(connection_string)
        self.parameters = {}
        self.running = True

        self.connection.wait_heartbeat()
        print("Heartbeat received from system (system %u component %u)" % (
            self.connection.target_system, self.connection.target_component))

        self.receiver_thread = threading.Thread(target=self.receive_messages)
        self.receiver_thread.start()

    def receive_messages(self):
        while self.running:
            msg = self.connection.recv_match(blocking=True, timeout=1)
            if msg:
                self.update_parameters(msg)

    def update_parameters(self, msg):
        msg_type = msg.get_type()
        if msg_type in [
            "ALTITUDE",
            "ATTITUDE",
            "BATTERY_STATUS",
            "GPS_RAW_INT",
            "GLOBAL_POSITION_INT",
            "LOCAL_POSITION_NED",
            "VFR_HUD",
            "SERVO_OUTPUT_RAW",
            "EXTENDED_SYS_STATE",
            "HOME_POSITION",
            "HEARTBEAT",
            "STATUS_TEXT",
            "SYS_STATUS",
            "ODOMETRY"
        ]:
            self.parameters[msg_type] = msg.to_dict()

    def get_parameters(self):
        return self.parameters

    def check_preflight(self):

        while True:
            msg = self.connection.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
            if msg:
                if msg.errors_count1 == 0:
                    print("Preflight check passed.")
                    break
                else:
                    print("Preflight check failed. Waiting for conditions to improve...")
            time.sleep(1)

    def set_com_arm_wo_gps(self):

        self.connection.mav.param_set_send(
            self.connection.target_system,
            self.connection.target_component,
            b'COM_ARM_WO_GPS',
            2,
            mavutil.mavlink.MAV_PARAM_TYPE_INT32
        )
        msg = self.connection.recv_match(type='PARAM_VALUE', blocking=True)
        print(f"Parameter COM_ARM_WO_GPS set to: {msg.param_value}")

    def arm(self):

        self.set_com_arm_wo_gps()

        self.check_preflight()

        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0,
            0
        )

    def takeoff(self, altitude):

        self.arm()

        time.sleep(2)

        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            altitude
        )

    def land(self):

        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0, 0, 0,
            0
        )

    def return_to_launch(self):

        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,
            0, 0, 0, 0, 0, 0,
            0
        )

    def reposition(self, latitude, longitude, altitude):

        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            30,
            0,
            0,
            0,
            0,
            latitude,
            longitude,
            altitude
        )
        msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)

    def stop(self):
        self.running = False
        self.receiver_thread.join()
        self.connection.close()
