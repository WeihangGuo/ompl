#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/trampoline.h>

#include "ompl/control/planners/syclop/GridDecomposition.h"
#include "ompl/control/planners/syclop/Decomposition.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/control/planners/syclop/Syclop.h"
#include "ompl/control/planners/rrt/RRT.h"
#include "ompl/control/planners/syclop/SyclopEST.h"
#include "ompl/base/Planner.h"

namespace nb = nanobind;

class PyGridDecomposition : public ompl::control::GridDecomposition {
public:
    NB_TRAMPOLINE(ompl::control::GridDecomposition, 2);

    void project(const ompl::base::State *s, std::vector<double> &coord) const override {
        NB_OVERRIDE_PURE(project, s, coord);
    }

    void sampleFullState(const ompl::base::StateSamplerPtr &sampler, const std::vector<double> &coord, ompl::base::State *s) const override {
        NB_OVERRIDE_PURE(sampleFullState, sampler, coord, s);
    }
};

void initPlanners(nb::module_& m) {   
    nb::class_<ompl::control::Decomposition>(m, "Decomposition");
    // nb::class_<ompl::control::GridDecomposition, ompl::control::Decomposition>(m, "_GridDecomposition");
    nb::class_<ompl::control::GridDecomposition, PyGridDecomposition /* <-- trampoline */>(m, "GridDecomposition")
        .def(nb::init<int, int, const ompl::base::RealVectorBounds&>())
        .def("project", &ompl::control::GridDecomposition::project)
        .def("sampleFullState", &ompl::control::GridDecomposition::sampleFullState)
        .def("getBounds", &ompl::control::GridDecomposition::getBounds)
        .def("getDimension", &ompl::control::GridDecomposition::getDimension);

    nb::class_<ompl::base::Planner>(m, "Planner");
    nb::class_<ompl::control::Syclop, ompl::base::Planner>(m, "Syclop");
    nb::class_<ompl::control::SyclopEST, ompl::control::Syclop>(m, "SyclopEST")
        .def(nb::init<const ompl::control::SpaceInformationPtr&, const std::shared_ptr<ompl::control::GridDecomposition>>());

    nb::class_<ompl::control::RRT, ompl::base::Planner>(m, "RRT")
        .def(nb::init<const ompl::control::SpaceInformationPtr&>());
}
