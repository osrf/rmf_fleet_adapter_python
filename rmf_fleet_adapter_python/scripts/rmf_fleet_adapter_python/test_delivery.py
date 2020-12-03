import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

import rmf_adapter as adpt
import rmf_adapter.vehicletraits as traits
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph
import rmf_adapter.plan as plan

from rmf_fleet_adapter_python.test_utils import MockRobotCommand
from rmf_fleet_adapter_python.test_utils import MockDispenser, MockIngestor
from rmf_fleet_adapter_python.test_utils import TaskSummaryObserver

from functools import partial


test_name = 'test_delivery'
map_name = "test_map"
fleet_name = "test_fleet"

pickup_name = "pickup"
dropoff_name = "dropoff"

dispenser_name = "mock_dispenser"
ingestor_name = "mock_ingestor"


def main():
    # INIT RCL ================================================================
    rclpy.init()

    try:
        adpt.init_rclcpp()
    except RuntimeError:
        # Continue if it is already initialized
        pass

    # INIT GRAPH ==============================================================
    test_graph = graph.Graph()

    test_graph.add_waypoint(map_name, [0.0, -10.0])  # 0
    test_graph.add_waypoint(map_name, [0.0, -5.0])  # 1
    test_graph.add_waypoint(map_name, [5.0, -5.0]).set_holding_point(True)  # 2
    test_graph.add_waypoint(map_name, [-10.0, 0])  # 3
    test_graph.add_waypoint(map_name, [-5.0, 0.0])  # 4
    test_graph.add_waypoint(map_name, [0.0, 0.0])  # 5
    test_graph.add_waypoint(map_name, [5.0, 0.0])  # 6
    test_graph.add_waypoint(map_name, [10.0, 0.0])  # 7
    test_graph.add_waypoint(map_name, [0.0, 5.0])  # 8
    test_graph.add_waypoint(map_name, [5.0, 5.0]).set_holding_point(True)  # 9
    test_graph.add_waypoint(map_name, [0.0, 10.0])  # 10

    assert test_graph.get_waypoint(2).holding_point
    assert test_graph.get_waypoint(9).holding_point
    assert not test_graph.get_waypoint(10).holding_point

    test_graph_legend = \
        """
        D : Dispenser
        I : Ingestor
        H : Holding Point
        Numerals : Waypoints
        ---- : Lanes
        """

    test_graph_vis = \
        test_graph_legend + \
        """
                         10(I)
                          |
                          |
                          8------9(H)
                          |      |
                          |      |
            3------4------5------6------7(D)
                          |      |
                          |      |
                          1------2(H)
                          |
                          |
                          0
       """
    print(test_graph_vis)

    test_graph.add_bidir_lane(0, 1)  # 0   1
    test_graph.add_bidir_lane(1, 2)  # 2   3
    test_graph.add_bidir_lane(1, 5)  # 4   5
    test_graph.add_bidir_lane(2, 6)  # 6   7
    test_graph.add_bidir_lane(3, 4)  # 8   9
    test_graph.add_bidir_lane(4, 5)  # 10 11
    test_graph.add_bidir_lane(5, 6)  # 12 13
    test_graph.add_dock_lane(6, 7, "A")  # 14 15
    test_graph.add_bidir_lane(5, 8)  # 16 17
    test_graph.add_bidir_lane(6, 9)  # 18 19
    test_graph.add_bidir_lane(8, 9)  # 20 21
    test_graph.add_dock_lane(8, 10, "B")  # 22 23

    assert test_graph.num_lanes == 24

    test_graph.add_key(pickup_name, 7)
    test_graph.add_key(dropoff_name, 10)

    assert len(test_graph.keys) == 2 and pickup_name in test_graph.keys \
        and dropoff_name in test_graph.keys

    # INIT FLEET ==============================================================
    profile = traits.Profile(geometry.make_final_convex_circle(1.0))
    robot_traits = traits.VehicleTraits(linear=traits.Limits(0.7, 0.3),
                                        angular=traits.Limits(1.0, 0.45),
                                        profile=profile)

    # Manages delivery or loop requests
    adapter = adpt.MockAdapter("TestDeliveryAdapter")
    fleet = adapter.add_fleet(fleet_name, robot_traits, test_graph)
    fleet.accept_delivery_requests(lambda x: True)

    cmd_node = Node("RobotCommandHandle")

    # Test compute_plan_starts, which tries to place the robot on the navgraph
    # Your robot MUST be near a waypoint or lane for this to work though!
    starts = plan.compute_plan_starts(test_graph,
                                      map_name,
                                      [[-10.0], [0.0], [0.0]],
                                      adapter.now())
    assert [x.waypoint for x in starts] == [3], [x.waypoint for x in starts]

    # Alternatively, if you DO know where your robot is, place it directly!
    starts = [plan.Start(adapter.now(),
                         0,
                         0.0)]

    # Lambda to insert an adapter
    def updater_inserter(handle_obj, updater):
        handle_obj.updater = updater

    # Manages and executes robot commands
    robot_cmd = MockRobotCommand(cmd_node, test_graph)

    fleet.add_robot(robot_cmd,
                    "T0",
                    profile,
                    starts,
                    partial(updater_inserter, robot_cmd))

    # INIT DISPENSERS =========================================================
    dispenser = MockDispenser(dispenser_name)
    ingestor = MockIngestor(ingestor_name)

    # INIT TASK SUMMARY OBSERVER ==============================================
    observer = TaskSummaryObserver()

    # FINAL PREP ==============================================================
    rclpy_executor = SingleThreadedExecutor()
    rclpy_executor.add_node(cmd_node)
    rclpy_executor.add_node(dispenser)
    rclpy_executor.add_node(ingestor)
    rclpy_executor.add_node(observer)

    # GO! =====================================================================
    adapter.start()
    print("# SENDING NEW DELIVERY REQUEST ###################################")
    request = adpt.type.CPPDeliveryMsg(test_name,
                                       pickup_name,
                                       dispenser_name,
                                       dropoff_name,
                                       ingestor_name)
    dispenser.reset()
    ingestor.reset()
    observer.reset()
    observer.add_task(test_name)
    adapter.request_delivery(request)
    rclpy_executor.spin_once(1)

    for i in range(1000):
        if observer.all_tasks_complete():
            print("Delivery Task Complete.")
            break
        rclpy_executor.spin_once(1)
        # time.sleep(0.2)

    results = observer.count_successful_tasks()
    print("\n== DEBUG TASK REPORT ==")
    print("Visited waypoints:", robot_cmd.visited_waypoints)
    print(f"Sucessful Tasks: {results[0]} / {results[1]}")

    assert results[0] == results[1], "Not all tasks were completed."

    error_msg = "Robot did not take the expected route"
    assert all([
        x in robot_cmd.visited_waypoints for x in [5, 6, 7, 8, 10]]), error_msg
    assert robot_cmd.visited_waypoints[5] == 4, error_msg
    assert robot_cmd.visited_waypoints[6] == 3, error_msg
    assert robot_cmd.visited_waypoints[7] == 1, error_msg
    assert robot_cmd.visited_waypoints[8] == 2, error_msg
    assert robot_cmd.visited_waypoints[10] == 1, error_msg

    cmd_node.destroy_node()
    observer.destroy_node()
    dispenser.destroy_node()
    ingestor.destroy_node()

    rclpy_executor.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
