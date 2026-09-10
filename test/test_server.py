import importlib.util
import sys
import types
import unittest
from collections import deque
from pathlib import Path
from unittest.mock import patch


def load_server_module():
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = object
    rclpy.node = rclpy_node

    package = types.ModuleType("ros2_robotiqgripper")
    services = types.ModuleType("ros2_robotiqgripper.srv")
    services.RobotiqGripper = object
    package.srv = services

    modules = {
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "ros2_robotiqgripper": package,
        "ros2_robotiqgripper.srv": services,
    }
    path = Path(__file__).parents[1] / "python" / "server.py"
    spec = importlib.util.spec_from_file_location("robotiq_server", path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


server = load_server_module()


class FakeSocket:
    def __init__(self, replies):
        self.replies = deque(reply.encode("ascii") for reply in replies)
        self.sent = []
        self.connected_to = None
        self.closed = False

    def __enter__(self):
        return self

    def __exit__(self, _exception_type, _exception, _traceback):
        self.closed = True

    def settimeout(self, _timeout):
        pass

    def connect(self, address):
        self.connected_to = address

    def sendall(self, data):
        self.sent.append(data.decode("ascii"))

    def recv(self, _size):
        if not self.replies:
            raise AssertionError("The test did not provide enough socket replies")
        return self.replies.popleft()


def active_request_replies(start_position, final_position):
    return [
        "ACT 1\n", "STA 3\n", "GTO 1\n", "FLT 0\n",
        f"POS {start_position}\n", "SPE 20\n", "FOR 20\n", "FLT 0\n",
        f"PRE {start_position}\n", "OBJ 3\n", "ack\n",
        f"PRE {final_position}\n", "OBJ 0\n",
        f"POS {start_position}\n", "FLT 0\n",
        f"PRE {final_position}\n", "OBJ 3\n",
        f"POS {final_position}\n", "FLT 0\n",
    ]


def inactive_request_replies(start_position, final_position):
    return [
        "ACT 0\n", "STA 0\n", "GTO 1\n", "FLT 0\n",
        f"POS {start_position}\n", "ack\n", "ack\n", "STA 3\n", "FLT 0\n",
        "SPE 20\n", "FOR 20\n", "ack\n", "FLT 0\n",
        f"PRE {start_position}\n", "OBJ 3\n", "ack\n",
        f"PRE {final_position}\n", "OBJ 0\n",
        f"POS {start_position}\n", "FLT 0\n",
        f"PRE {final_position}\n", "OBJ 3\n",
        f"POS {final_position}\n", "FLT 0\n",
    ]


class ServerStateTests(unittest.TestCase):
    def test_already_ready_gripper_is_not_reactivated(self):
        gripper = FakeSocket([
            "ACT 1\n", "STA 3\n", "GTO 1\n", "FLT 0\n", "POS 127\n",
            "SPE 20\n", "FOR 20\n", "FLT 0\n",
        ])

        server.prepare_gripper(gripper)

        self.assertEqual(
            gripper.sent[:5],
            ["GET ACT\n", "GET STA\n", "GET GTO\n", "GET FLT\n", "GET POS\n"],
        )
        self.assertNotIn("SET ACT 1\n", gripper.sent)
        self.assertFalse(any(command.startswith("SET ") for command in gripper.sent))

    def test_close_uses_obj_when_pre_confirms_target(self):
        gripper = FakeSocket([
            "PRE 0\n", "OBJ 3\n", "ack\n",
            "PRE 255\n", "OBJ 3\n", "POS 227\n", "FLT 0\n",
        ])

        final_position = server.move_gripper(gripper, 255)

        self.assertEqual(final_position, 227)

    def test_open_uses_obj_when_pre_confirms_target(self):
        gripper = FakeSocket([
            "PRE 255\n", "OBJ 3\n", "ack\n",
            "PRE 0\n", "OBJ 3\n", "POS 3\n", "FLT 0\n",
        ])

        final_position = server.move_gripper(gripper, 0)

        self.assertEqual(final_position, 3)

    def test_stale_obj_three_with_previous_pre_is_not_accepted(self):
        gripper = FakeSocket([
            "PRE 0\n", "OBJ 3\n", "ack\n",
            "PRE 0\n", "OBJ 3\n", "POS 3\n", "FLT 0\n",
            "PRE 255\n", "OBJ 0\n", "POS 100\n", "FLT 0\n",
            "PRE 255\n", "OBJ 3\n", "POS 227\n", "FLT 0\n",
        ])

        final_position = server.move_gripper(gripper, 255)

        self.assertEqual(final_position, 227)
        self.assertFalse(gripper.replies)

    def test_repeated_already_reached_target_succeeds(self):
        gripper = FakeSocket([
            "PRE 255\n", "OBJ 3\n", "ack\n",
            "PRE 255\n", "OBJ 3\n", "POS 227\n", "FLT 0\n",
        ])

        final_position = server.move_gripper(gripper, 255)

        self.assertEqual(final_position, 227)

    def test_inactive_gripper_is_activated(self):
        gripper = FakeSocket([
            "ACT 0\n", "STA 0\n", "GTO 1\n", "FLT 0\n", "POS 0\n",
            "ack\n", "ack\n", "STA 3\n", "FLT 0\n",
            "SPE 20\n", "FOR 20\n", "ack\n", "FLT 0\n",
        ])

        server.prepare_gripper(gripper)

        self.assertIn("SET GTO 0\n", gripper.sent)
        self.assertIn("SET ACT 1\n", gripper.sent)
        self.assertNotIn("SET ACT 0\n", gripper.sent)

    def test_fault_five_is_temporary_during_activation(self):
        gripper = FakeSocket([
            "ACT 0\n", "STA 0\n", "GTO 0\n", "FLT 0\n", "POS 0\n",
            "ack\n", "ack\n", "STA 1\n", "FLT 5\n", "STA 3\n", "FLT 0\n",
            "SPE 20\n", "FOR 20\n", "ack\n", "FLT 0\n",
        ])

        server.prepare_gripper(gripper)

        self.assertEqual(gripper.sent.count("SET ACT 1\n"), 1)

    def test_ready_state_with_fault_five_gets_one_recovery(self):
        gripper = FakeSocket([
            "ACT 1\n", "STA 3\n", "GTO 1\n", "FLT 5\n", "POS 3\n",
            "ack\n", "ack\n", "ACT 0\n", "STA 0\n", "FLT 5\n",
            "ack\n", "STA 1\n", "FLT 5\n", "STA 3\n", "FLT 0\n",
            "SPE 20\n", "FOR 20\n", "ack\n", "FLT 0\n",
        ])

        server.prepare_gripper(gripper)

        self.assertEqual(gripper.sent.count("SET GTO 0\n"), 1)
        self.assertEqual(gripper.sent.count("SET ACT 0\n"), 1)
        self.assertEqual(gripper.sent.count("SET ACT 1\n"), 1)

    def test_consecutive_requests_do_not_reactivate(self):
        first = FakeSocket(inactive_request_replies(0, 255))
        second = FakeSocket(active_request_replies(255, 0))
        service = types.SimpleNamespace(ip="192.0.2.1")
        close_request = types.SimpleNamespace(action="CLOSE")
        open_request = types.SimpleNamespace(action="OPEN")
        close_response = types.SimpleNamespace()
        open_response = types.SimpleNamespace()

        with patch.object(server.socket, "socket", side_effect=[first, second]), \
                patch.object(server.time, "sleep"):
            server.serviceServer.ExecuteService(
                service, close_request, close_response
            )
            server.serviceServer.ExecuteService(
                service, open_request, open_response
            )

        self.assertTrue(close_response.success)
        self.assertTrue(open_response.success)
        self.assertEqual(close_response.value, 255)
        self.assertEqual(open_response.value, 0)
        self.assertEqual(first.sent.count("SET ACT 1\n"), 1)
        self.assertNotIn("SET ACT 1\n", second.sent)
        self.assertEqual(first.connected_to, ("192.0.2.1", 63352))
        self.assertEqual(second.connected_to, ("192.0.2.1", 63352))
        self.assertTrue(first.closed)
        self.assertTrue(second.closed)


if __name__ == "__main__":
    unittest.main()
