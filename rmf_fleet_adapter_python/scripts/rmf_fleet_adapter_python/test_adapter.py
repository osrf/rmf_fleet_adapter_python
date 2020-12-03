import rmf_dispenser_msgs.msg as dispenser_msgs
import rmf_ingestor_msgs.msg as ingestor_msgs
from rmf_task_msgs.msg import TaskSummary

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

import rmf_adapter as adpt
import rmf_adapter.vehicletraits as traits
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph
import rmf_adapter.plan as plan

import datetime
import time
from functools import partial

from collections import namedtuple

###############################################################################
# PARAMS
###############################################################################

pickup_name = "pickup"
dropoff_name = "dropoff"

dispenser_name = "mock_dispenser"
ingestor_name = "mock_ingestor"

###############################################################################
# CONSTS
###############################################################################

INGESTOR_RESULT_ACKNOWLEDGED = 0
INGESTOR_RESULT_SUCCESS = 1
INGESTOR_RESULT_FAILED = 2

INGESTOR_STATE_IDLE = 0
INGESTOR_STATE_BUSY = 1
INGESTOR_STATE_OFFLINE = 2

DISPENSER_RESULT_ACKNOWLEDGED = 0
DISPENSER_RESULT_SUCCESS = 1
DISPENSER_RESULT_FAILED = 2

DISPENSER_STATE_IDLE = 0
DISPENSER_STATE_BUSY = 1
DISPENSER_STATE_OFFLINE = 2

TASK_STATE_QUEUED = 0
TASK_STATE_ACTIVE = 1
TASK_STATE_COMPLETED = 2
TASK_STATE_FAILED = 3


###############################################################################
# CLASSES
###############################################################################

# using these data structures for better readability
# request_guid in this case is that of the requester
DispenserTask = namedtuple("DispenserTask", "request_guid receive_time")
IngestorTask = namedtuple("IngestorTask", "request_guid receive_time")


class MockDispenser(Node):
    # Data structure to represent a DispenserTask
    def __init__(self, name, dispense_duration_sec=1.0,
                 publish_states=True, publish_results=True):
        super().__init__(name)

        # Variables
        self.reset(name)
        self.dispense_duration_sec = float(dispense_duration_sec)
        self.publish_states = publish_states
        self.publish_results = publish_results

        self.current_request_idx = 0  # Points to current task
        self.tasks = []  # Ordered Tasks by time of receipt

        # Currently only one task can be processed at a time
        self.timer = self.create_timer(
            0.1,
            self._timer_cb
        )

        # Pub-sub
        self.result_pub = self.create_publisher(
            dispenser_msgs.DispenserResult,
            'dispenser_results',
            1
        )
        self.state_pub = self.create_publisher(
            dispenser_msgs.DispenserState,
            'dispenser_states',
            1
        )
        self.request_sub = self.create_subscription(
            dispenser_msgs.DispenserRequest,
            'dispenser_requests',
            self._process_request_cb,
            1
        )

    def reset(self, name=None):
        # reset sets a new name if provided
        if name is not None:
            self.name = name

        self.tasks = []

    def _process_request_cb(self, msg):
        # Process request only if addressed to this dispenser
        if msg.target_guid != self.name:
            return

        task_already_received = [
            x for x in self.tasks if x.request_guid == msg.request_guid]

        if task_already_received:
            # print("[MockDispenser] DUPLICATE REQUEST")
            pass
        else:
            # New request to process
            print("[MockDispenser] NEW REQUEST RECEIVED")
            new_task = DispenserTask(
                request_guid=msg.request_guid,
                receive_time=self.get_clock().now().to_msg())
            self.tasks.append(new_task)

    def _timer_cb(self):
        # Report Dispenser State
        if self.publish_states:
            state = dispenser_msgs.DispenserState()
            state.time = self.get_clock().now().to_msg()
            state.guid = self.name
            if self.tasks:
                state.request_guid_queue = [
                    x.request_guid for x in self.tasks][
                        self.current_request_idx:]
            if state.request_guid_queue:
                state.mode = DISPENSER_STATE_BUSY
            else:
                state.mode = DISPENSER_STATE_IDLE
            state.seconds_remaining = self.dispense_duration_sec
            self.state_pub.publish(state)

        if self.tasks:
            if self.current_request_idx >= len(self.tasks):
                # No more new tasks, do nothing
                return
        else:
            # Tasks is empty, return
            return

        # If enough time has elapsed, consider dispensing done
        time_now = self.get_clock().now().to_msg().sec
        time_elapsed = time_now - self.tasks[
            self.current_request_idx].receive_time.sec
        if time_elapsed > self.dispense_duration_sec:
            if self.publish_results:
                # Send a success message
                result = dispenser_msgs.DispenserResult()
                result.time = self.get_clock().now().to_msg()
                result.status = DISPENSER_RESULT_SUCCESS
                result.source_guid = self.name
                result.request_guid = self.tasks[
                    self.current_request_idx].request_guid
                self.result_pub.publish(result)
            self.current_request_idx += 1


class MockIngestor(Node):
    def __init__(self, name, ingest_duration_sec=1,
                 publish_states=True, publish_results=True):
        super().__init__(name)

        # Variables
        self.reset(name)
        self.ingest_duration_sec = float(ingest_duration_sec)
        self.publish_states = publish_states
        self.publish_results = publish_results

        self.current_request_idx = 0  # Points to current task
        self.tasks = []  # Ordered Tasks by time of receipt

        # Currently only one task can be processed at a time
        self.timer = self.create_timer(
            0.1,
            self._timer_cb
        )

        # Pub-sub
        self.result_pub = self.create_publisher(
            ingestor_msgs.IngestorResult,
            'ingestor_results',
            1
        )
        self.state_pub = self.create_publisher(
            ingestor_msgs.IngestorState,
            'ingestor_states',
            1
        )
        self.request_sub = self.create_subscription(
            ingestor_msgs.IngestorRequest,
            'ingestor_requests',
            self._process_request_cb,
            1
        )

    def reset(self, name=None):
        # reset sets a new name if provided
        if name is not None:
            self.name = name

        self.tasks = []

    def _process_request_cb(self, msg):
        # Process request only if addressed to this Ingestor
        if msg.target_guid != self.name:
            return

        task_already_received = [
            x for x in self.tasks if x.request_guid == msg.request_guid]

        if task_already_received:
            # print("[MockIngestor] DUPLICATE REQUEST")
            pass
        else:
            # New request to process
            print("[MockIngestor] NEW REQUEST RECEIVED")
            new_task = IngestorTask(
                request_guid=msg.request_guid,
                receive_time=self.get_clock().now().to_msg())
            self.tasks.append(new_task)

    def _timer_cb(self):
        # Report Ingestor State
        if self.publish_states:
            state = ingestor_msgs.IngestorState()
            state.time = self.get_clock().now().to_msg()
            state.guid = self.name
            if self.tasks:
                state.request_guid_queue = [
                    x.request_guid for x in self.tasks][
                        self.current_request_idx:]
            if state.request_guid_queue:
                state.mode = INGESTOR_STATE_BUSY
            else:
                state.mode = INGESTOR_STATE_IDLE
            state.seconds_remaining = self.ingest_duration_sec
            self.state_pub.publish(state)

        if self.tasks:
            if self.current_request_idx >= len(self.tasks):
                # No more new tasks, do nothing
                return
        else:
            # Tasks is empty, return
            return

        # If enough time has elapsed, consider ingesting done
        time_now = self.get_clock().now().to_msg().sec
        time_elapsed = time_now - self.tasks[
            self.current_request_idx].receive_time.sec
        if time_elapsed > self.ingest_duration_sec:
            if self.publish_results:
                # Send a success message
                result = ingestor_msgs.IngestorResult()
                result.time = self.get_clock().now().to_msg()
                result.status = INGESTOR_RESULT_SUCCESS
                result.source_guid = self.name
                result.request_guid = self.tasks[
                    self.current_request_idx].request_guid
                self.result_pub.publish(result)
            self.current_request_idx += 1


class MockRobotCommand(adpt.RobotCommandHandle):
    class EventListener(graph.lane.Executor):
        def __init__(self, dock_to_wp, wp):
            graph.lane.Executor.__init__(self)

            self.dock_to_wp = dock_to_wp
            self.wp = wp

        def dock_execute(self, dock):
            self.dock_to_wp[dock.dock_name] = self.wp
            print("DOCK EVENT EXECUTED FOR DOCK:", dock.dock_name)

        # And these are all overrided but meant to do nothing
        def door_open_execute(self, door_open):
            return

        def door_close_execute(self, door_close):
            return

        def lift_door_open_execute(self, lift_door_open):
            return

        def lift_door_close_execute(self, lift_door_close):
            return

        def lift_move_execute(self, lift_move):
            return

    def __init__(self, node, graph):
        adpt.RobotCommandHandle.__init__(self)

        self.updater = None

        self.active = False
        self.node = node
        self.timer = None
        self.current_waypoint_target = 0
        self.dockings = {}
        self.visited_waypoints = {}
        self.dock_to_wp = {}

        for i in range(graph.num_lanes):
            lane = graph.get_lane(i)

            # lane.entry and lane.exit are a Lane::Node wrappers
            if lane.entry.event:
                executor = self.EventListener(self.dock_to_wp,
                                              lane.exit.waypoint_index)
                try:
                    lane.entry.event.execute(executor)
                except Exception:
                    print(type(lane.entry.event))
                    print("EVENT EXECUTE FOR LANE", i, "NOT IMPLEMENTED")

        print("Registered Docks:", self.dock_to_wp, "\n")

    def follow_new_path(self,
                        waypoints,
                        next_arrival_estimator,  # function!
                        path_finished_callback):
        print("\n[RobotCommandHandle] Setting new path of %d waypoints..."
              % len(waypoints))
        print("Waypoints:", [x.graph_index for x in waypoints])

        self.stop()

        self.current_waypoint_target = 0
        self.active = True
        self.timer = self.node.create_timer(
            0.01,
            partial(self._timer_cb,
                    waypoints=waypoints,
                    next_arrival_estimator=next_arrival_estimator,
                    path_finished_callback=path_finished_callback)
        )

    def stop(self):
        try:
            self.timer.reset()
            self.timer.cancel()
        except Exception:
            # For when a timer does not exist yet
            pass

    def dock(self, dock_name, docking_finished_callback):
        assert dock_name in self.dock_to_wp

        # For both dockings and visited waypoints, increment the associated
        # keyed values by 1. Or start it off at 1 if it doesn't exist yet.
        self.dockings[dock_name] = self.dockings.get(dock_name, 0) + 1

        waypoint = self.dock_to_wp[dock_name]
        self.visited_waypoints[waypoint] = (
            self.visited_waypoints.get(waypoint, 0) + 1)

        docking_finished_callback()
        print("[RobotCommandHandle] DOCKING FINISHED")

    def _timer_cb(self,
                  waypoints,
                  next_arrival_estimator,
                  path_finished_callback):
        if not self.active:
            return

        if self.current_waypoint_target < len(waypoints):
            self.current_waypoint_target += 1

        if self.updater:
            # This waypoint is a plan waypoint, NOT graph waypoint!!
            previous_waypoint = waypoints[self.current_waypoint_target - 1]

            if previous_waypoint.graph_index:
                print("[RobotUpdateHandle] UPDATING ROBOT POSITION:",
                      previous_waypoint.graph_index)

                self.updater.update_position(
                    previous_waypoint.graph_index,
                    previous_waypoint.position[2]
                )
                self.visited_waypoints[previous_waypoint.graph_index] = (
                    self.visited_waypoints
                        .get(previous_waypoint.graph_index, 0)
                    + 1
                )
            else:
                print("[RobotUpdateHandle] UPDATING ROBOT POSITION DEFAULT:",
                      previous_waypoint.position)
                # TODO(CH3): NOTE(CH3): Confirm this magic string is wanted
                self.updater.update_position("test_map",
                                             previous_waypoint.position)

        if self.current_waypoint_target < len(waypoints):
            # Again, this waypoint is a plan waypoint! NOT a graph waypoint!!
            waypoint = waypoints[self.current_waypoint_target]
            test_delay = (datetime.timedelta(milliseconds=750)
                          * self.current_waypoint_target)

            node_time = self.node.get_clock().now().nanoseconds / 1e9
            now = datetime.datetime.fromtimestamp(node_time)

            delayed_arrival_time = waypoint.time + test_delay

            # Note: next_arrival_estimator
            # expects a std::chrono::duration,
            # not a std::chrono::steady_clock::time_point

            # The duration represents the time delay from
            # self.node.get_clock().now() till the arrival time,
            # and NOT the time_point that represents that arrival time!!!
            next_arrival_estimator(self.current_waypoint_target,
                                   delayed_arrival_time - now)
        else:
            self.active = False
            self.timer.reset()

            path_finished_callback()
            print("[RobotCommandHandle] PATH FINISHED")


class TaskSummaryObserver(Node):
    def __init__(self):
        super().__init__('task_observer')
        # Maps task_ids to True/False Completions
        self.tasks_status = {}

        # Pub-sub
        self.request_sub = self.create_subscription(
            TaskSummary,
            'task_summaries',
            self._process_task_summary_cb,
            1
        )

    def all_tasks_complete(self):
        return all(list(self.tasks_status.values()))
 
    def add_task(self, task_name):
        self.tasks_status[task_name] = False

    def count_successful_tasks(self):
        successful = 0
        statuses = self.tasks_status.values()
        for status_complete in statuses:
            if status_complete:
                successful += 1
        return (successful, len(statuses))

    def _process_task_summary_cb(self, msg):
        task_name = msg.task_id
        if task_name not in self.tasks_status.keys():
            print('Observed Unaccounted Task. This should not happen')
            return
        if msg.state == TASK_STATE_COMPLETED:
            self.tasks_status[task_name] = True


def main():
    # INIT RCL ================================================================
    rclpy.init()
    adpt.init_rclcpp()

    # INIT GRAPH ==============================================================
    map_name = "test_map"
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
    
    test_graph_vis = \
        """
        D - Dispenser
        I - Ingestor
                         10(I)
                          |
                          |
                          8------9
                          |      |
                          |      |
            3------4------5------6------7(D)
                          |      |
                          |      |
                          1------2
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
    fleet = adapter.add_fleet("test_fleet", robot_traits, test_graph)
    fleet.accept_delivery_requests(lambda x: True)

    cmd_node = Node("RobotCommandHandle")

    # Test compute_plan_starts, which tries to place the robot on the navgraph
    # Your robot MUST be near a waypoint or lane for this to work though!
    starts = plan.compute_plan_starts(test_graph,
                                      "test_map",
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

    print("# SENDING NEW REQUEST ############################################")
    test_name = 'test_delivery'
    request = adpt.type.CPPDeliveryMsg(test_name,
                                       pickup_name,
                                       dispenser_name,
                                       dropoff_name,
                                       ingestor_name)
    dispenser.reset()
    ingestor.reset()
    observer.add_task(test_name)
    adapter.request_delivery(request)
    rclpy_executor.spin_once(1)

    for i in range(1000):
        if observer.all_tasks_complete():
            print("All Tasks Complete.")
            break
        rclpy_executor.spin_once(1)
        # time.sleep(0.2)

    results = observer.count_successful_tasks()
    print("\n== DEBUG TASK REPORT ==")
    print("Visited waypoints:", robot_cmd.visited_waypoints)
    print(f"Sucessful Tasks: {results[0]} / {results[1]}")

    assert results[0] == results[1]  # All tasks complete

    # Robot planned paths take the shortest path
    assert all([x in robot_cmd.visited_waypoints for x in [5, 6, 7, 8, 10]])
    assert robot_cmd.visited_waypoints[5] == 4
    assert robot_cmd.visited_waypoints[6] == 3
    assert robot_cmd.visited_waypoints[7] == 1
    assert robot_cmd.visited_waypoints[8] == 2
    assert robot_cmd.visited_waypoints[10] == 1

    # Old assertions
    # assert all([x in robot_cmd.visited_waypoints for x in [0, 5, 6, 7, 8, 10]])
    # assert len(robot_cmd.visited_waypoints) == 6
    # assert robot_cmd.visited_waypoints[0] == 2
    # assert robot_cmd.visited_waypoints[5] == 4
    # assert robot_cmd.visited_waypoints[6] == 3
    # assert robot_cmd.visited_waypoints[7] == 1
    # assert robot_cmd.visited_waypoints[8] == 2
    # assert robot_cmd.visited_waypoints[10] == 1

    cmd_node.destroy_node()
    observer.destroy_node()
    dispenser.destroy_node()
    ingestor.destroy_node()

    rclpy_executor.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
