/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/eigen.h>
#include <pybind11/chrono.h>
#include <pybind11/stl.h>
#include <optional>

#include <memory>
#include <random>
#include <cstdint>

#include <rmf_utils/impl_ptr.hpp>
#include <rmf_utils/clone_ptr.hpp>
#include <rmf_utils/optional.hpp>
#include <rmf_traffic/Time.hpp>

#include <rmf_task_msgs/msg/delivery.hpp>
#include <rmf_task_msgs/msg/loop.hpp>
#include <rmf_task_msgs/msg/task_profile.hpp>

namespace py = pybind11;

using Duration = rmf_traffic::Duration;
using TaskType = rmf_task_msgs::msg::TaskType;
using Loop = rmf_task_msgs::msg::Loop;
using Delivery = rmf_task_msgs::msg::Delivery;
using TaskDescription = rmf_task_msgs::msg::TaskDescription;

Delivery make_delivery_msg(
    std::string task_id,
    std::string pickup_place_name,
    std::string pickup_dispenser,
    std::string dropoff_place_name,
    std::string dropoff_ingestor)
{
    Delivery request;
    request.task_id = task_id;

    request.pickup_place_name = pickup_place_name;
    request.pickup_dispenser = pickup_dispenser;

    request.dropoff_place_name = dropoff_place_name;
    request.dropoff_ingestor = dropoff_ingestor;

    return request;
}

Loop make_loop_msg(
    std::string task_id,
    std::string robot_type,
    uint32_t num_loops,
    std::string start_name,
    std::string finish_name)
{
    Loop request;
    request.task_id = task_id;
    request.robot_type = robot_type;
    request.num_loops = num_loops;

    request.start_name = start_name;
    request.finish_name = finish_name;

    return request;
}


void bind_types(py::module &m)
{
    auto m_type = m.def_submodule("type");

    // Wrapper for the std::mt19937 class
    py::class_<std::mt19937>(m_type, "mt19937")
        .def(py::init<int>())
        .def(py::init<unsigned int>())
        .def_static("min", &std::mt19937::min)
        .def_static("max", &std::mt19937::max)
        // .def("seed", &std::mt19937::seed) // Doesn't exist for some reason
        .def("discard", &std::mt19937::discard)
        .def("__call__", &std::mt19937::operator());

    py::class_<Delivery>(m_type, "CPPDeliveryMsg")
        .def(py::init(&make_delivery_msg),
             py::arg("task_id") = "",
             py::arg("pickup_place_name") = "",
             py::arg("pickup_dispenser") = "",
             py::arg("dropoff_place_name") = "",
             py::arg("dropoff_ingestor") = "")
        .def_property(
            "task_id",
            [&](Delivery &self) { return self.task_id; },
            [&](Delivery &self,
                std::string task_id) { self.task_id = task_id; })
        .def_property(
            "pickup_place_name",
            [&](Delivery &self) { return self.pickup_place_name; },
            [&](Delivery &self,
                std::string pickup_place_name) { self.pickup_place_name = pickup_place_name; })
        .def_property(
            "pickup_dispenser",
            [&](Delivery &self) { return self.pickup_dispenser; },
            [&](Delivery &self,
                std::string pickup_dispenser) { self.pickup_dispenser = pickup_dispenser; })
        .def_property(
            "dropoff_place_name",
            [&](Delivery &self) { return self.dropoff_place_name; },
            [&](Delivery &self,
                std::string dropoff_place_name) { self.dropoff_place_name = dropoff_place_name; })
        .def_property(
            "dropoff_ingestor",
            [&](Delivery &self) { return self.dropoff_ingestor; },
            [&](Delivery &self,
                std::string dropoff_ingestor) { self.dropoff_ingestor = dropoff_ingestor; });

    py::class_<Loop>(m_type, "CPPLoopMsg")
        .def(py::init(&make_loop_msg),
             py::arg("task_id") = "",
             py::arg("robot_type") = "",
             py::arg("num_loops") = "",
             py::arg("start_name") = "",
             py::arg("finish_name") = "")
        .def_property(
            "task_id",
            [&](Loop &self) { return self.task_id; },
            [&](Loop &self,
                std::string task_id) { self.task_id = task_id; })
        .def_property(
            "robot_type",
            [&](Loop &self) { return self.robot_type; },
            [&](Loop &self,
                std::string robot_type) { self.robot_type = robot_type; })
        .def_property(
            "num_loops",
            [&](Loop &self) { return self.num_loops; },
            [&](Loop &self,
                uint32_t num_loops) { self.num_loops = num_loops; })
        .def_property(
            "start_name",
            [&](Loop &self) { return self.start_name; },
            [&](Loop &self,
                std::string start_name) { self.start_name = start_name; })
        .def_property(
            "finish_name",
            [&](Loop &self) { return self.finish_name; },
            [&](Loop &self,
                std::string finish_name) { self.finish_name = finish_name; });

    // TODO: Support More elements (e.g. start_time, loop.....)!
    py::class_<TaskDescription>(m_type, "CPPTaskDescriptionMsg")
        .def(py::init<>())
        .def_property(
            "delivery",
            [&](TaskDescription &self) { return self.delivery; },
            [&](TaskDescription &self, Delivery delivery) {
                        self.task_type.type = TaskType::TYPE_DELIVERY;
                        self.delivery = delivery; },
            "Delivery Task, this will ovewrite the prev type")
        .def_property(
            "loop",
            [&](TaskDescription &self) { return self.loop; },
            [&](TaskDescription &self, Loop loop) {
                        self.task_type.type = TaskType::TYPE_LOOP;
                        self.loop = loop; },
            "Loop Task, this will ovewrite the prev type");
}
