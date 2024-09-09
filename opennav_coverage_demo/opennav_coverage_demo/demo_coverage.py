#! /usr/bin/env python3
# Copyright 2023 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from enum import Enum
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, Polygon
from lifecycle_msgs.srv import GetState
from opennav_coverage_msgs.action import NavigateCompleteCoverage
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node


class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class CoverageNavigatorTester(Node):

    def __init__(self):
        super().__init__(node_name='coverage_navigator_tester')
        self.goal_handle = None
        self.result_future = None
        self.status = None
        self.feedback = None

        self.coverage_client = ActionClient(self, NavigateCompleteCoverage,
                                            'navigate_complete_coverage')

    def destroy_node(self):
        self.coverage_client.destroy()
        super().destroy_node()

    def toPolygon(self, field):
        poly = Polygon()
        for coord in field:
            pt = Point32()
            pt.x = coord[0]
            pt.y = coord[1]
            poly.points.append(pt)
        return poly

    def navigateCoverage(self, field, obstacle):
        """Send a `NavToPose` action request."""
        print("Waiting for 'NavigateCompleteCoverage' action server")
        while not self.coverage_client.wait_for_server(timeout_sec=1.0):
            print('"NavigateCompleteCoverage" action server not available, waiting...')

        goal_msg = NavigateCompleteCoverage.Goal()
        goal_msg.frame_id = 'map'
        goal_msg.polygons.append(self.toPolygon(field))

        if len(obstacle)>0:
            for obs in obstacle:
                goal_msg.polygons.append(self.toPolygon(obs))

        print('Navigating to with field of size: ' + str(len(field)) + '...')
        send_goal_future = self.coverage_client.send_goal_async(goal_msg,
                                                                self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            print('Navigate Coverage request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def isTaskComplete(self):
        """Check if the task request of any type is complete yet."""
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                print(f'Task with failed with status code: {self.status}')
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        print('Task succeeded!')
        return True

    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        return

    def getFeedback(self):
        """Get the pending action feedback message."""
        return self.feedback

    def getResult(self):
        """Get the pending action result message."""
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return TaskResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return TaskResult.CANCELED
        else:
            return TaskResult.UNKNOWN

    def startup(self, node_name='bt_navigator'):
        # Waits for the node within the tester namespace to become active
        print(f'Waiting for {node_name} to become active..')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            print(f'{node_service} service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            print(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                print(f'Result of get_state: {state}')
            time.sleep(2)
        return


def main():
    rclpy.init()

    navigator = CoverageNavigatorTester()
    navigator.startup()

    # Some example field
    # field = [[5.0, 5.0], [5.0, 15.0], [15.0, 15.0], [10.0, 5.0], [5.0, 5.0]]
    # obstacle = [
    #     [[10.0, 10.0], [10.0, 13.0], [13.0, 13.0], [13.0, 10.0], [10.0, 10.0]], 
    #     [[8.0,8.0],[8.0,9.0],[9.0,9.0],[9.0,9.0], [8.0,8.0]]
    #     ]
    field = [[2.9, 20.5], [2.3, 18.8], [2.2, 17.4], [1.9, 15.6], [1.4, 13.7], [1.0, 12.1], [0.0, 10.8], [0.5, 10.0], [1.5, 9.8], [2.5, 9.6], [3.5, 9.2], [4.1, 8.1], [4.0, 7.0], [3.9, 6.0], [4.3, 5.8], [5.2, 6.0], [6.6, 5.8], [8.2, 5.7], [9.8, 5.5], [11.5, 5.3], [13.0, 5.0], [14.2, 5.2], [14.1, 6.1], [14.3, 7.5], [14.8, 8.6], [15.9, 9.0], [16.9, 8.5], [17.7, 8.1], [18.2, 7.1], [18.1, 6.0], [18.2, 4.8], [19.0, 4.0], [20.3, 3.5], [21.9, 3.3], [23.6, 3.3], [25.3, 3.0], [27.1, 2.7], [29.0, 2.6], [30.9, 2.5], [32.8, 2.2], [34.7, 2.0], [36.8, 1.7], [38.8, 1.4], [40.8, 1.3], [42.9, 1.2], [44.9, 0.9], [47.0, 0.7], [49.2, 0.5], [51.2, 0.3], [53.1, 0.1], [54.8, 0.5], [56.0, 1.5], [56.6, 2.9], [58.1, 3.9], [59.6, 3.6], [60.3, 2.6], [60.4, 1.3], [60.9, 0.8], [62.6, 0.7], [64.5, 0.3], [66.3, 0.0], [67.9, 0.0], [68.6, 0.5], [68.7, 2.1], [68.9, 4.1], [69.6, 6.0], [71.1, 6.4], [71.8, 6.9], [71.7, 8.3], [71.5, 10.2], [71.9, 11.9], [72.1, 13.6], [71.7, 14.2], [70.5, 14.6], [69.8, 15.8], [69.5, 16.8], [68.7, 17.5], [67.2, 17.4], [65.2, 17.2], [63.9, 16.9], [63.2, 15.5], [62.7, 13.6], [61.4, 12.0], [59.6, 11.0], [57.6, 10.7], [55.6, 10.9], [53.6, 11.1], [51.7, 11.2], [49.7, 11.3], [47.7, 11.5], [45.8, 12.0], [44.7, 13.6], [44.7, 15.3], [44.7, 17.1], [43.8, 17.9], [42.0, 18.0], [40.1, 18.1], [37.8, 18.3], [35.7, 18.6], [34.0, 18.8], [33.4, 18.3], [33.2, 16.6], [32.6, 14.8], [30.8, 13.6], [28.7, 13.3], [26.7, 13.6], [24.7, 14.2], [22.7, 14.6], [20.8, 14.7], [18.8, 14.7], [17.2, 15.6], [15.9, 16.6], [15.1, 18.0], [14.7, 19.6], [14.6, 21.2], [14.3, 22.6], [13.5, 24.3], [13.4, 26.4], [13.3, 28.2], [12.3, 29.0], [10.4, 29.5], [8.3, 29.7], [6.1, 29.6], [4.2, 29.5], [2.8, 29.1], [2.6, 27.7], [2.5, 25.8], [2.4, 24.0], [2.2, 22.3], [2.9, 20.5]]

    obstacle = [[[36.7, 6.2], [36.7, 6.2], [36.8, 6.2], [38.0, 5.9], [39.5, 5.7], [41.0, 5.5], [42.5, 5.3], [43.9, 5.1], [44.6, 5.3], [44.8, 6.3], [44.8, 7.7], [44.9, 9.1], [44.8, 10.4], [43.9, 11.1], [42.5, 11.5], [40.8, 12.0], [39.2, 12.2], [37.7, 12.2], [36.7, 11.8], [36.1, 10.5], [35.8, 9.2], [35.7, 7.6], [35.9, 6.4], [36.4, 6.2], [36.7, 6.2]]]

    navigator.navigateCoverage(field, obstacle)

    i = 0
    while not navigator.isTaskComplete():
        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')
        time.sleep(1)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')


if __name__ == '__main__':
    main()
