#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/control/SpaceInformation.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/StateSpace.h"

namespace nb = nanobind;

void initSpaceInformation(nb::module_& m) {    
    nb::class_<ompl::control::SpaceInformation, ompl::base::SpaceInformation>(m, "_SpaceInformation")
        .def("setPropagationStepSize", &ompl::control::SpaceInformation::setPropagationStepSize);
    m.def("SpaceInformation", [](const ompl::base::StateSpacePtr &ss, ompl::control::ControlSpacePtr &cs) { return std::make_shared<ompl::control::SpaceInformation>(ss, cs); }, nb::rv_policy::reference);
}