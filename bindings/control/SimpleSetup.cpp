#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/function.h>

#include "ompl/control/SimpleSetup.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/control/ControlSpace.h"

#include "ompl/base/State.h"
#include "ompl/base/ScopedState.h"
#include "ompl/base/spaces/SE2StateSpace.h"
namespace nb = nanobind;

void initSimpleSetup(nb::module_& m) {    
    nb::class_<ompl::control::SimpleSetup>(m, "SimpleSetup")
        // Constructors
        .def(nb::init<const ompl::control::SpaceInformationPtr&>())
        .def(nb::init<const ompl::control::ControlSpacePtr&>())
        
        // Space/State Information getters
        .def("getSpaceInformation", &ompl::control::SimpleSetup::getSpaceInformation, nb::rv_policy::reference)
        .def("getStateSpace", &ompl::control::SimpleSetup::getStateSpace, nb::rv_policy::reference)
        .def("getControlSpace", &ompl::control::SimpleSetup::getControlSpace, nb::rv_policy::reference)
        .def("getProblemDefinition", static_cast<const ompl::base::ProblemDefinitionPtr& (ompl::control::SimpleSetup::*)() const>(&ompl::control::SimpleSetup::getProblemDefinition))
        
        // State/Goal setters
        .def("setStartAndGoalStates", [](ompl::control::SimpleSetup& self, 
                                        const ompl::base::ScopedState<ompl::base::SE2StateSpace>& start, 
                                        const ompl::base::ScopedState<ompl::base::SE2StateSpace>& goal, 
                                        double threshold = std::numeric_limits<double>::epsilon()) {
            self.setStartAndGoalStates(start, goal, threshold);
        }, nb::arg("start"), nb::arg("goal"), nb::arg("threshold") = std::numeric_limits<double>::epsilon())
        .def("setStartState", &ompl::control::SimpleSetup::setStartState)
        .def("setGoalState", &ompl::control::SimpleSetup::setGoalState,
             nb::arg("goal"), nb::arg("threshold") = std::numeric_limits<double>::epsilon())
        .def("addStartState", &ompl::control::SimpleSetup::addStartState)
        .def("clearStartStates", &ompl::control::SimpleSetup::clearStartStates)
        
        // Planner methods
        .def("setPlanner", &ompl::control::SimpleSetup::setPlanner)
        .def("getPlanner", &ompl::control::SimpleSetup::getPlanner)
        
        // State validity and propagation
        .def("setStateValidityChecker", nb::overload_cast<const ompl::base::StateValidityCheckerFn&>(&ompl::control::SimpleSetup::setStateValidityChecker))
        .def("setStatePropagator", nb::overload_cast<const ompl::control::StatePropagatorFn&>(&ompl::control::SimpleSetup::setStatePropagator))
        
        // Solve methods
        .def("solve", 
            nb::overload_cast<double>(&ompl::control::SimpleSetup::solve),
            nb::arg("time")
        )        
        // Solution checking
        .def("haveSolutionPath", &ompl::control::SimpleSetup::haveSolutionPath)
        .def("haveExactSolutionPath", &ompl::control::SimpleSetup::haveExactSolutionPath)
        .def("getSolutionPath", &ompl::control::SimpleSetup::getSolutionPath, nb::rv_policy::reference)
        
        // Setup and utility
        .def("setup", &ompl::control::SimpleSetup::setup)
        .def("clear", &ompl::control::SimpleSetup::clear)
        .def("print", &ompl::control::SimpleSetup::print);
}