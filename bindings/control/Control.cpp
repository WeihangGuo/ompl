#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/control/Control.h"
#include "ompl/control/ControlSpace.h"

namespace nb = nanobind;

void initControl(nb::module_& m) {    
    nb::class_<ompl::control::ControlSpace>(m, "ControlSpace");
    nb::class_<ompl::control::Control>(m, "Control"); 
    nb::class_<ompl::control::CompoundControl, ompl::control::Control>(m, "CompoundControl");
}