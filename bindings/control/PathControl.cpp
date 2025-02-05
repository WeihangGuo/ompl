#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/string.h>
#include "ompl/control/PathControl.h"



namespace nb = nanobind;

void initPathControl(nb::module_& m) {    
    nb::class_<ompl::control::PathControl, ompl::base::Path>(m, "PathControl")
        .def("print", [](const ompl::control::PathControl &self) {
            std::ostringstream ss;
            self.print(ss);
            return ss.str();
        })
        .def("__str__", [](const ompl::control::PathControl &self) {
            std::ostringstream ss;
            self.print(ss);
            return ss.str();
        })
        .def("printAsMatrix", [](const ompl::control::PathControl &self) {
            std::ostringstream ss;
            self.printAsMatrix(ss);
            return ss.str();
        });
}