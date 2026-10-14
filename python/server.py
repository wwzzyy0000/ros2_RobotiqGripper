#!/usr/bin/python3

# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Fahad Khan       - f.khan@cranfield.ac.uk                                   #
#           Dr. Seemal Asif  - s.asif@cranfield.ac.uk                                   #
#           Prof. Phil Webb  - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: May, 2024.                                                                     #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2024) ROS 2 Robotiq Gripper Driver. URL: https://github.com/IFRA-Cranfield/ros2_RobotiqGripper.

# IMPORT -> Required libraries:
import rclpy
from rclpy.node import Node
from ros2_robotiqgripper.srv import RobotiqGripper
from sensor_msgs.msg import JointState
import socket, time, re, threading

SOCKET_TIMEOUT = 3.0
ACTIVATION_TIMEOUT = 10.0
MOVEMENT_TIMEOUT = 10.0
POLL_INTERVAL = 0.1
GRIPPER_PORT = 63352
IDLE_POLL_RATE_HZ = 10.0
GRIPPER_OPEN_POSITION = 3
GRIPPER_CLOSED_POSITION = 230
GRIPPER_POSITION_RANGE = GRIPPER_CLOSED_POSITION - GRIPPER_OPEN_POSITION
GRIPPER_CLOSED_JOINT_POSITION = 0.7929

JOINT_SUFFIXES = (
    "robotiq_85_left_knuckle_joint",
    "robotiq_85_right_knuckle_joint",
    "robotiq_85_left_inner_knuckle_joint",
    "robotiq_85_right_inner_knuckle_joint",
    "robotiq_85_left_finger_tip_joint",
    "robotiq_85_right_finger_tip_joint",
)
JOINT_SIGNS = (1.0, -1.0, 1.0, -1.0, -1.0, 1.0)


class GripperError(Exception):
    pass


def position_to_joint_angle(position):
    normalized = (position - GRIPPER_OPEN_POSITION) / GRIPPER_POSITION_RANGE
    normalized = max(0.0, min(normalized, 1.0))
    return GRIPPER_CLOSED_JOINT_POSITION * normalized


def joint_names_and_positions(joint_prefix, position):
    angle = position_to_joint_angle(position)
    names = [joint_prefix + suffix for suffix in JOINT_SUFFIXES]
    positions = [sign * angle for sign in JOINT_SIGNS]
    return names, positions


def receive_reply(gripper_socket):
    data = gripper_socket.recv(2**10)
    if not data:
        raise GripperError("The gripper closed the TCP connection without replying.")

    try:
        return data.decode("ascii").strip()
    except UnicodeDecodeError as error:
        raise GripperError("The gripper returned a non-ASCII reply.") from error


def set_variable(gripper_socket, variable, value):
    gripper_socket.sendall(f"SET {variable} {value}\n".encode("ascii"))
    reply = receive_reply(gripper_socket)
    if reply.lower() != "ack":
        raise GripperError(
            f"Unexpected reply to SET {variable} {value}: {reply!r}."
        )


def get_variable(gripper_socket, variable):
    gripper_socket.sendall(f"GET {variable}\n".encode("ascii"))
    reply = receive_reply(gripper_socket)
    match = re.fullmatch(rf"{re.escape(variable)}\s+(-?\d+)", reply)
    if match is None:
        raise GripperError(f"Malformed reply to GET {variable}: {reply!r}.")
    return int(match.group(1))


def raise_for_fault(fault, allowed_faults=()):
    if fault != 0 and fault not in allowed_faults:
        raise GripperError(f"The gripper reports fault code {fault}.")


def check_for_fault(gripper_socket):
    fault = get_variable(gripper_socket, "FLT")
    raise_for_fault(fault)


def read_initial_state(gripper_socket):
    return {
        variable: get_variable(gripper_socket, variable)
        for variable in ("ACT", "STA", "GTO", "FLT", "POS")
    }


def wait_for_deactivation(gripper_socket):
    deadline = time.monotonic() + ACTIVATION_TIMEOUT
    last_act = None
    last_status = None

    while time.monotonic() < deadline:
        last_act = get_variable(gripper_socket, "ACT")
        last_status = get_variable(gripper_socket, "STA")
        fault = get_variable(gripper_socket, "FLT")
        raise_for_fault(fault, allowed_faults=(5,))
        if last_act == 0 and last_status == 0:
            return
        time.sleep(POLL_INTERVAL)

    raise GripperError(
        "Timed out waiting for the activation reset "
        f"(last ACT value: {last_act}, last STA value: {last_status})."
    )


def wait_for_activation(gripper_socket):
    deadline = time.monotonic() + ACTIVATION_TIMEOUT
    last_status = None
    last_fault = None

    while time.monotonic() < deadline:
        last_status = get_variable(gripper_socket, "STA")
        last_fault = get_variable(gripper_socket, "FLT")
        raise_for_fault(last_fault, allowed_faults=(5,))
        if last_status == 3 and last_fault == 0:
            return
        time.sleep(POLL_INTERVAL)

    raise GripperError(
        "Timed out waiting for gripper activation "
        f"(last STA value: {last_status}, last FLT value: {last_fault})."
    )


def configure_gripper(gripper_socket, current_gto):
    speed = get_variable(gripper_socket, "SPE")
    force = get_variable(gripper_socket, "FOR")

    if speed != 20:
        set_variable(gripper_socket, "SPE", 20)
    if force != 20:
        set_variable(gripper_socket, "FOR", 20)
    if current_gto != 1:
        set_variable(gripper_socket, "GTO", 1)


def prepare_gripper(gripper_socket):
    state = read_initial_state(gripper_socket)
    already_ready = (
        state["ACT"] == 1 and state["STA"] == 3 and state["FLT"] == 0
    )

    if already_ready:
        current_gto = state["GTO"]
    else:
        raise_for_fault(state["FLT"], allowed_faults=(5,))

        set_variable(gripper_socket, "GTO", 0)
        if state["ACT"] != 0 or state["STA"] != 0:
            set_variable(gripper_socket, "ACT", 0)
            wait_for_deactivation(gripper_socket)

        set_variable(gripper_socket, "ACT", 1)
        wait_for_activation(gripper_socket)
        current_gto = 0

    configure_gripper(gripper_socket, current_gto)
    check_for_fault(gripper_socket)


def read_and_publish_position(gripper_socket, position_callback):
    position = get_variable(gripper_socket, "POS")
    position_callback(position)
    return position


def move_gripper(gripper_socket, target_position, position_callback=None):
    previous_target = get_variable(gripper_socket, "PRE")
    previous_object_status = get_variable(gripper_socket, "OBJ")
    if previous_object_status not in (0, 1, 2, 3):
        raise GripperError(
            f"The gripper returned invalid OBJ value {previous_object_status}."
        )

    set_variable(gripper_socket, "POS", target_position)

    deadline = time.monotonic() + MOVEMENT_TIMEOUT
    target_accepted = previous_target == target_position
    last_requested_position = previous_target
    last_object_status = None
    last_position = None
    while time.monotonic() < deadline:
        last_requested_position = get_variable(gripper_socket, "PRE")
        last_object_status = get_variable(gripper_socket, "OBJ")
        last_position = get_variable(gripper_socket, "POS")
        if position_callback is not None:
            position_callback(last_position)
        fault = get_variable(gripper_socket, "FLT")
        raise_for_fault(fault)

        if last_object_status not in (0, 1, 2, 3):
            raise GripperError(
                f"The gripper returned invalid OBJ value {last_object_status}."
            )

        if last_requested_position == target_position:
            target_accepted = True
        if target_accepted and last_object_status in (1, 2, 3):
            return last_position

        time.sleep(POLL_INTERVAL)

    raise GripperError(
        "Timed out waiting for gripper movement to finish "
        f"(last PRE value: {last_requested_position}, "
        f"last OBJ value: {last_object_status}, "
        f"last POS value: {last_position})."
    )

# ===== INPUT PARAMETER ===== #
PARAM_IP = "0.0.0.0"
P_CHECK_IP = False

class ipPARAM(Node):

    def __init__(self):

        global PARAM_IP
        global P_CHECK_IP

        super().__init__('ros2_robotiq_ip_param')
        self.declare_parameter('IPAddress', "None")

        PARAM_IP = self.get_parameter('IPAddress').get_parameter_value().string_value

        if (PARAM_IP == "None"):

            print('IPAddress ROS 2 parameter was not defined for the ros2_robotiq service server.')
            exit()

        else:
            print('IPAddress ROS 2 parameter received: ' + PARAM_IP)

        P_CHECK_IP = True

# Create NODE:
class serviceServer(Node):

    def __init__(self, IP):

        # Initialise ROS 2 service server:
        super().__init__('ros2_RobotiqGripper_ServiceServer')
        self.SERVICE = self.create_service(RobotiqGripper, "Robotiq_Gripper", self.ExecuteService)

        self.ip = IP
        self.declare_parameter('joint_prefix', '')
        self.declare_parameter('idle_poll_rate_hz', IDLE_POLL_RATE_HZ)
        self.joint_prefix = self.get_parameter(
            'joint_prefix').get_parameter_value().string_value
        idle_poll_rate_hz = self.get_parameter(
            'idle_poll_rate_hz').get_parameter_value().double_value

        if not self.joint_prefix:
            raise ValueError("The joint_prefix parameter must not be empty.")
        if idle_poll_rate_hz <= 0.0:
            raise ValueError("The idle_poll_rate_hz parameter must be positive.")

        self.joint_state_publisher = self.create_publisher(
            JointState, "/joint_states", 10)
        self.socket_lock = threading.Lock()
        self.position_timer = self.create_timer(
            1.0 / idle_poll_rate_hz, self.PollPosition)

    def PublishPosition(self, gripper_position):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name, message.position = joint_names_and_positions(
            self.joint_prefix, gripper_position)
        self.joint_state_publisher.publish(message)

    def PollPosition(self):
        # A non-blocking acquisition also protects this node if it is later moved
        # to a multi-threaded executor. Service commands always take precedence.
        if not self.socket_lock.acquire(blocking=False):
            return

        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as gripper_socket:
                gripper_socket.settimeout(SOCKET_TIMEOUT)
                gripper_socket.connect((self.ip, GRIPPER_PORT))
                read_and_publish_position(gripper_socket, self.PublishPosition)
        except (GripperError, OSError) as error:
            # Do not publish the last known position with a new timestamp. The
            # next timer invocation will try to obtain a fresh sample instead.
            self.get_logger().warning(
                "Unable to read a fresh Robotiq gripper position: " + str(error),
                throttle_duration_sec=5000,
            )
        finally:
            self.socket_lock.release()

    def ExecuteService(self, request, response):

        # INITIALISE RESPONSE:
        response.success = False
        response.value = -1
        response.average = -1.0

        if request.action not in ("OPEN", "CLOSE"):
            response.message = "ERROR: Valid commands are OPEN/CLOSE. Please try again."
            return(response)

        target_position = 255 if request.action == "CLOSE" else 0

        try:
            with self.socket_lock:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as gripper_socket:
                    gripper_socket.settimeout(SOCKET_TIMEOUT)
                    gripper_socket.connect((self.ip, GRIPPER_PORT))

                    prepare_gripper(gripper_socket)
                    gripper_position = move_gripper(
                        gripper_socket,
                        target_position,
                        self.PublishPosition,
                    )
        except (GripperError, OSError) as error:
            response.message = (
                "ERROR: Communication with Robotiq gripper failed: " + str(error)
            )
            return(response)

        average = round((float(gripper_position)/255.0)*100.0, 2)
        response.success = True
        response.value = gripper_position
        response.average = average
        response.message = (
            f"{request.action} command successfully sent to Robotiq gripper. "
            f"After execution, the gripper is -> {average}% CLOSED."
        )
        return(response)

# =================== MAIN =================== #
def main(args=None):

    rclpy.init(args=args)

    # Get IP Address:PROGRAM
    global PARAM_IP
    global P_CHECK_IP

    paramNODE = ipPARAM()
    while (P_CHECK_IP == False):
        rclpy.spin_once(paramNODE)
    paramNODE.destroy_node()

    # Initialise NODE:
    GripperNode = serviceServer(PARAM_IP)
    print ("[ROS 2 Robotiq Gripper]: ros2_RobotiqGripper_ServiceServer generated.")

    # Spin service:
    rclpy.spin(GripperNode)

    GripperNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
