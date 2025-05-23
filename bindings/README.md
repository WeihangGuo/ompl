# OMPL Python Bindings

## How to use Python binding

The Python bindings mirror OMPL’s C++ API. For detailed usage, refer to:

- The [C++ OMPL documentation](https://ompl.kavrakilab.org/)  
- The sample scripts in the `pyexamples/` directory  

## How is Nanobind organized?
We use **Nanobind** to generate Python bindings for OMPL. For every OMPL header, there is a corresponding binding file in the `bindings/` directory, organized into subfolders that mirror OMPL’s module structure. For example, all bindings for `ompl::base` live under `bindings/base`. 
The [Nanobind documentation](https://nanobind.readthedocs.io/en/latest/) provides a comprehensive guide to writing bindings. The one thing to watch out for is [overriding C++ virtual methods in Python](https://nanobind.readthedocs.io/en/latest/classes.html#overriding-virtual-functions-in-python)—you need a **trampoline class** to forward calls correctly.

## How to contribute to the bindings
In this Beta release, some binding components are still unimplemented or untested and are marked with `TODO` comments following this standard convention:

```C++
// TODO [<SCOPE>][<CATEGORY>][<OPTIONAL_SUBTAG>]
//
// <SCOPE>:
//   ClassName   // class name
//
// <CATEGORY>:
//   IMPLEMENT    // feature not yet implemented
//   TEST         // missing tests
//   TRAMPOLINE   // virtual function needing a Python trampoline
//   MISSING      // (class-only) functions that must be added
//
// <OPTIONAL_SUBTAG>:
//   For MISSING:    <missingFunctionName>[, <anotherName>]
//
// Examples:
// TODO [ob::StateSpace][MISSING]["ParamSet & params ()", "void clearStateSamplerAllocator ()"]
// TODO [ob::Planner][TRAMPOLINE]
// TODO [ob::OwenStateSpace][IMPLEMENT]
// TODO [oc::SST][TEST]

```
Since the Python bindings are still under development and OMPL exposes a vast API, we may have overlooked some object-ownership details. If you encounter a double-free error when using the bindings, please investigate the ownership annotations. Nanobind provides comprehensive guides on [object ownership](https://nanobind.readthedocs.io/en/latest/ownership.html) and [advanced usage](https://nanobind.readthedocs.io/en/latest/ownership_adv.html#enable-shared-from-this). In most cases, applying the rv_policy::reference_internal return-value policy to your binding will resolve this issue.

## Note on ARM platforms
Generating bindings on ARM platforms (e.g., Apple M-series) can sometimes fail due to compiler/linker incompatibilities. To work around this:

- Use **`gcc`** (instead of the system default) when building the OMPL bindings.  
- If you run into Boost-related linker errors, download and build Boost from source with **`gcc`** as well.  

