# Copyright 2020 Open Source Robotics Foundation, Inc.
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

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

import rmf_adapter as adpt
import rmf_adapter.vehicletraits as traits
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph
import rmf_adapter.plan as plan

import rmf_adapter.easy_traffic_light as traffic_light

from rmf_fleet_adapter_python.test_utils import MockRobotCommand
from rmf_fleet_adapter_python.test_utils import TaskSummaryObserver

import time
from functools import partial

class MockTrafficLightHandle:
    def __init__(self):
        print("Mock EZ traffic light")
        self.handler = None
        self.hihi = "init"
        pass

    def traffic_light_cb(self, ez_traffic_light):
        print("Add traffic light handler!")
        self.handler = ez_traffic_light

    def pause_cb(self):
        print("Iam told to pause")
        pass

    def resume_cb(self):
        print("Iam told to resume")
        pass

    def pub_follow_path(self):
        print("Pub follow path")
        path = []
        path.append(adpt.Waypoint("test_map", [0,0,0], 0.1, True ))
        path.append(adpt.Waypoint("test_map", [0,1,0], 0.1, True ))
        path.append(adpt.Waypoint("test_map", [0,2,0], 0.1, True ))
        path.append(adpt.Waypoint("test_map", [2,2,0], 0.1, True ))
        self.handler.follow_new_path(path)

    def move_robot(self, checkpoint, waypoint):
        for _ in range(20):
            result = self.handler.waiting_at(checkpoint)
            time.sleep(0.3)

            print("check move? ", result)
            if (result == traffic_light.WaitingInstruction.Resume):
                print("moving bot")
                break

        time.sleep(1)
        result = self.handler.moving_from(checkpoint, waypoint)
        print("moving result: ", result)

def main():
    rclpy.init()
    adpt.init_rclcpp()
    profile = traits.Profile(geometry.make_final_convex_circle(1.0))
    robot_traits = traits.VehicleTraits(linear=traits.Limits(0.7, 0.3),
                                        angular=traits.Limits(1.0, 0.45),
                                        profile=profile)

    print("Test Run Easy Traffic Light")
    mock_tl = MockTrafficLightHandle()
    adapter = adpt.Adapter.make("TestTrafficLightAdapter")
    adapter.add_easy_traffic_light(
        mock_tl.traffic_light_cb,
        "dummybot_fleet",
        "dummybot",
        robot_traits,
        mock_tl.pause_cb,
        mock_tl.resume_cb)

    adapter.start()
    time.sleep(1)

    # Test Command
    mock_tl.pub_follow_path()
    mock_tl.move_robot(0, [0,1,0])
    mock_tl.move_robot(1, [0,2,0])

    rclpy.shutdown()

if __name__ == "__main__":
    main()
