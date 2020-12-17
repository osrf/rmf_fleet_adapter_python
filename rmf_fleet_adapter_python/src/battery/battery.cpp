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

#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/agv/SimpleMotionPowerSink.hpp>
#include <rmf_battery/agv/SimpleDevicePowerSink.hpp>

namespace py = pybind11;
namespace agv = rmf_battery::agv;

void bind_battery(py::module &m)
{
  auto m_battery = m.def_submodule("battery");

  py::class_<agv::BatterySystem>(m_battery, "BatterySystem")
      .def_static("make", &agv::BatterySystem::make,
                  py::arg("nominal_voltage"),
                  py::arg("capacity"),
                  py::arg("charging_current"))
      .def_property_readonly("get_nominal_voltage",
        &agv::BatterySystem::nominal_voltage)
      .def_property_readonly("get_capacity",
        &agv::BatterySystem::capacity)
      .def_property_readonly("get_charging_current",
        &agv::BatterySystem::charging_current);

  // motion sink
  py::class_<agv::SimpleMotionPowerSink>(m_battery, "SimpleMotionPowerSink")
      // NOTE: MechanicalSystem is being abstracted
      // .def(py::init<agv::BatterySystem&, agv::MechanicalSystem&>())
      .def(py::init(
          [&](agv::BatterySystem& battery_system, 
              double mass, double inertia, double friction_coefficient )
        {
          auto mechanical_system = agv::MechanicalSystem::make(
            mass, inertia, friction_coefficient);
          return std::unique_ptr<agv::SimpleMotionPowerSink>(
              new agv::SimpleMotionPowerSink(
                battery_system, *mechanical_system));
        }),
        py::arg("battery_system"),
        py::arg("mass"),
        py::arg("inertia"),
        py::arg("friction_coefficient"),
         "Note: MechanicalSystem is being abstracted")
      .def_property_readonly("get_battery_system",
           &agv::SimpleMotionPowerSink::battery_system)
      // .def_property_readonly("get_mechanical_system", 
      //      &agv::SimpleMotionPowerSink::mechanical_system)
      .def_property_readonly("get_mass",
        [&](agv::SimpleMotionPowerSink &self){
          return self.mechanical_system().mass();
        })
      .def_property_readonly("get_inertia",
        [&](agv::SimpleMotionPowerSink &self){
          return self.mechanical_system().inertia();
        })
      .def_property_readonly("get_friction_coefficient",
        [&](agv::SimpleMotionPowerSink &self){
          return self.mechanical_system().friction_coefficient();
        });

  // ambient sink and tool sink
  py::class_<agv::SimpleDevicePowerSink>(m_battery, "SimpleDevicePowerSink")
      // NOTE: PowerSystem is being abstracted
      // .def(py::init<agv::BatterySystem&, agv::PowerSystem&>())
      .def(py::init(
          [&](agv::BatterySystem& battery_system, double nominal_power )
        {
          auto power_system = agv::PowerSystem::make(nominal_power);
          return std::unique_ptr<agv::SimpleDevicePowerSink>(
              new agv::SimpleDevicePowerSink(battery_system, *power_system));
        }),
        py::arg("battery_system"),
        py::arg("power_system_nominal_voltage"),
        "Note: PowerSystem is being abstracted")
      .def_property_readonly("get_battery_system",
        &agv::SimpleDevicePowerSink::battery_system)
      // .def_property_readonly("get_power_system", 
      //   &agv::SimpleDevicePowerSink::power_system)
      .def_property_readonly("get_norminal_power",
        [&](agv::SimpleDevicePowerSink &self){
          return self.power_system().nominal_power();
        });
}
