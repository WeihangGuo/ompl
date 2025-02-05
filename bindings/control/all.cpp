#include <nanobind/nanobind.h>

namespace nb = nanobind;

// Forward declarations
void initRealVectorControlSpace(nb::module_&);
void initSimpleSetup(nb::module_&);
void initPlanners(nb::module_&);
void initSpaceInformation(nb::module_&);
void initControl(nb::module_&); 
void initPathControl(nb::module_&);

NB_MODULE(control, m) {
    initControl(m);
    initRealVectorControlSpace(m);
    initSimpleSetup(m);
    initSpaceInformation(m);
    initPlanners(m);
    initPathControl(m);
}