#!/usr/bin/env python3
"""Sequentially load / configure / activate ros2_control controllers.

Stock ``spawner`` often times out under Gazebo dual-robot load (CM answers
slowly). This tool polls with long timeouts and never gives up early on a
controller that is already loaded but still unconfigured/inactive.
"""

from __future__ import annotations

import time
from typing import Dict, List, Optional

import rclpy
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    LoadController,
    SwitchController,
)
from rclpy.node import Node


class ControllerActivator(Node):
    def __init__(self) -> None:
        super().__init__('controller_activator')
        self.declare_parameter('controller_manager', '/trailer_controller_manager')
        self.declare_parameter(
            'controllers', ['joint_state_broadcaster', 'diff_drive_controller'])
        self.declare_parameter('timeout_sec', 300.0)
        self.declare_parameter('poll_period_sec', 2.0)
        self.declare_parameter('service_wait_sec', 45.0)
        self.declare_parameter('call_timeout_sec', 45.0)

        self._cm = str(self.get_parameter('controller_manager').value)
        self._controllers: List[str] = list(self.get_parameter('controllers').value)
        self._timeout = float(self.get_parameter('timeout_sec').value)
        self._poll = float(self.get_parameter('poll_period_sec').value)
        self._service_wait = float(self.get_parameter('service_wait_sec').value)
        self._call_timeout = float(self.get_parameter('call_timeout_sec').value)

        self._list = self.create_client(ListControllers, f'{self._cm}/list_controllers')
        self._load = self.create_client(LoadController, f'{self._cm}/load_controller')
        self._configure = self.create_client(
            ConfigureController, f'{self._cm}/configure_controller')
        self._switch = self.create_client(SwitchController, f'{self._cm}/switch_controller')

        # Controllers we already asked to load (avoids spam if CM is slow).
        self._load_requested = set()

        self.get_logger().info(
            f'Will activate {self._controllers} on {self._cm} '
            f'(timeout={self._timeout:.0f}s, call_timeout={self._call_timeout:.0f}s)'
        )

    def _call(self, client, request, label: str) -> Optional[object]:
        if not client.wait_for_service(timeout_sec=self._service_wait):
            self.get_logger().warn(f'{label}: service not available yet')
            return None
        future = client.call_async(request)
        # Spin until done or timeout (wall clock).
        end = time.monotonic() + self._call_timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            if future.done():
                break
        if not future.done():
            self.get_logger().warn(f'{label}: timed out after {self._call_timeout:.0f}s')
            return None
        try:
            return future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'{label}: {exc}')
            return None

    def _states(self) -> Dict[str, str]:
        result = self._call(self._list, ListControllers.Request(), 'list_controllers')
        if result is None:
            return {}
        return {c.name: c.state for c in result.controller}

    def _advance_one(self, name: str, state: str) -> bool:
        """Return True if this controller is active."""
        if state == 'active':
            return True

        if state == 'missing':
            if name in self._load_requested:
                self.get_logger().info(
                    f'{name}: load already requested, waiting for CM…')
                return False
            req = LoadController.Request()
            req.name = name
            self.get_logger().info(f'Loading {name}…')
            res = self._call(self._load, req, f'load {name}')
            self._load_requested.add(name)
            if res is not None and getattr(res, 'ok', False):
                self.get_logger().info(f'Loaded {name}')
            else:
                # Timed out or already-loaded: next poll will see real state.
                self.get_logger().info(
                    f'{name}: load call unfinished/failed — will re-check state')
            return False

        if state == 'unconfigured':
            req = ConfigureController.Request()
            req.name = name
            self.get_logger().info(f'Configuring {name}…')
            res = self._call(self._configure, req, f'configure {name}')
            if res is not None and getattr(res, 'ok', False):
                self.get_logger().info(f'Configured {name}')
            return False

        if state == 'inactive':
            req = SwitchController.Request()
            req.activate_controllers = [name]
            req.start_controllers = [name]
            req.deactivate_controllers = []
            req.stop_controllers = []
            req.strictness = SwitchController.Request.BEST_EFFORT
            req.start_asap = False
            req.activate_asap = False
            req.timeout.sec = int(self._call_timeout)
            req.timeout.nanosec = 0
            self.get_logger().info(f'Activating {name}…')
            res = self._call(self._switch, req, f'activate {name}')
            if res is not None and getattr(res, 'ok', False):
                self.get_logger().info(f'Activated {name}')
            return False

        self.get_logger().warn(f'{name}: unexpected state "{state}"')
        return False

    def run(self) -> int:
        deadline = time.monotonic() + self._timeout
        self.get_logger().info(f'Waiting for {self._cm}…')

        while rclpy.ok() and time.monotonic() < deadline:
            states = self._states()
            if not states and not self._list.service_is_ready():
                time.sleep(self._poll)
                continue

            all_active = True
            # Process one controller at a time (JSB before diff_drive).
            for name in self._controllers:
                state = states.get(name, 'missing')
                if not self._advance_one(name, state):
                    all_active = False
                    break  # finish this one before the next
                # Refresh states after a successful step
                states = self._states() or states

            if all_active and all(
                states.get(n) == 'active' for n in self._controllers
            ):
                self.get_logger().info(
                    f'All controllers active on {self._cm}: {self._controllers}')
                return 0

            # Re-check after poll; clear load flag if state appeared
            for name in self._controllers:
                if states.get(name, 'missing') != 'missing':
                    self._load_requested.discard(name)

            time.sleep(self._poll)

        self.get_logger().error(
            f'Deadline reached — not all controllers active on {self._cm}. '
            f'Last states: {self._states()}')
        return 1


def main() -> None:
    rclpy.init()
    node = ControllerActivator()
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
