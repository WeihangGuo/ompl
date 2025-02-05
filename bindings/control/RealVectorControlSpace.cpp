#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/operators.h>
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/control/ControlSpace.h"

namespace nb = nanobind;

void initRealVectorControlSpace(nb::module_ &m) {
    nb::class_<ompl::control::RealVectorControlSpace::ControlType, ompl::control::Control>(m, "RealVectorControl")
        .def("__getitem__", [](const ompl::control::RealVectorControlSpace::ControlType &control, unsigned int i) { return control[i]; })
        .def("__setitem__", [](ompl::control::RealVectorControlSpace::ControlType &control, unsigned int i, double v) { control[i] = v; });
    nb::class_<ompl::control::RealVectorControlSpace, ompl::control::ControlSpace>(m, "RealVectorControlSpace")
        .def(nb::init<const ompl::base::StateSpacePtr&, unsigned int>())
        .def("setBounds", &ompl::control::RealVectorControlSpace::setBounds)
        .def("getBounds", &ompl::control::RealVectorControlSpace::getBounds, nb::rv_policy::reference)
        .def("getDimension", &ompl::control::RealVectorControlSpace::getDimension)
        .def("allocControl", &ompl::control::RealVectorControlSpace::allocControl)
        .def("freeControl", &ompl::control::RealVectorControlSpace::freeControl)
        .def("nullControl", &ompl::control::RealVectorControlSpace::nullControl)
        .def("copyControl", &ompl::control::RealVectorControlSpace::copyControl)
        .def("equalControls", &ompl::control::RealVectorControlSpace::equalControls)
        .def("setup", &ompl::control::RealVectorControlSpace::setup);
}