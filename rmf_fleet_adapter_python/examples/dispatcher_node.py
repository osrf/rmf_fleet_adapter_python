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

import time
import rmf_adapter as adpt
import rmf_adapter.type as Type
import rmf_adapter.nodes as Nodes

from threading import Thread


def main():
    adpt.init_rclcpp()

    print("Creating Simple Dispatcher Node")
    dispatcher = Nodes.DispatcherNode.make("sample_dispatcher_node")

    # submit a dummy task
    def submit_task(mod):
        time.sleep(2)
        profile = Type.CPPTaskProfileMsg()
        profile.delivery = adpt.type.CPPDeliveryMsg()
        print("initated Task")
        id1 = mod.submit_task(profile)
        id2 = mod.submit_task(profile)
        print(f" active list >>  {mod.get_active_task_ids()}")

        time.sleep(3)
        state1 = mod.get_task_state(id1)
        state2 = mod.get_task_state(id2)

        print(f"state: Task 1 > {state1}, Task 2 > {state2}")
        print("Done")

    print("spinning dispatcher")
    th1 = Thread(target=submit_task, args=(dispatcher,))
    th1.start()

    print("spinning rclcpp")
    while True:
        adpt.spin_some_rclcpp(dispatcher.node())
        time.sleep(0.1)

    print("Exiting")


if __name__ == "__main__":
    main()
