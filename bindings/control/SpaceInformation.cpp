#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/control/SpaceInformation.h"

namespace nb = nanobind;

void initSpaceInformation(nb::module_& m) {    
    nb::class_<ompl::control::SpaceInformation>(m, "SpaceInformation")
        .def("setPropagationStepSize", &ompl::control::SpaceInformation::setPropagationStepSize);
}