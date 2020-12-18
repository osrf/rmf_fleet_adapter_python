#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/iostream.h>
#include <pybind11/eigen.h>
#include <pybind11/chrono.h>
#include <pybind11/stl.h>
#include <optional>

#include <memory>
#include <random>
#include <cstdint>

#include <rmf_traffic_ros2/blockade/Node.hpp>
#include <rmf_traffic_ros2/schedule/Node.hpp>
#include <rmf_task_ros2/dispatcher/Dispatcher.hpp>

namespace py = pybind11;
using Dispatcher = rmf_task_ros2::dispatcher::Dispatcher;


// TODO (YL) Might place this at a different repo as this is not a part of adapter
void bind_nodes(py::module &m) {
  auto m_nodes = m.def_submodule("nodes");

  // Make blockade_Node
  m_nodes.def("make_blockade", &rmf_traffic_ros2::blockade::make_node,
              py::arg("options"));

  // Make Schedule Node
  m_nodes.def("make_schedule", &rmf_traffic_ros2::schedule::make_node,
              py::arg("options"));

  /// Dispatcher Node Class
  /// \brief Create a simple dispatcher node api mainly for testing
  ///
  py::class_<Dispatcher, std::shared_ptr<Dispatcher>>(
          m_nodes, "DispatcherNode")
    .def_static("make", &Dispatcher::make,
          py::arg("dispatcher_node_name"),
          py::call_guard<py::scoped_ostream_redirect,
                        py::scoped_estream_redirect>())
    .def("submit_task", &Dispatcher::submit_task,
          py::arg("task_profile"))
    .def("spin", &Dispatcher::spin,
      "This is a blocking spin")
    .def("node", &Dispatcher::node)
    .def("get_active_task_ids", [](Dispatcher& self) {     
            std::vector<std::string> task_ids;
            for (auto task : *(self.active_tasks()))
            {
              task_ids.push_back(task.first);
            }  
            return task_ids;
          })
    .def("get_terminated_task_ids", [](Dispatcher& self) {
            std::vector<std::string> task_ids;
            for (auto task : *(self.terminated_tasks()))
            {
              task_ids.push_back(task.first);
            }  
            return task_ids;
          })
    .def("get_task_state", [](Dispatcher& self, std::string& task_id) {
            return (int)*(self.get_task_state(task_id));
          },
          py::arg("task_id"));
}
