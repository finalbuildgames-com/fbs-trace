# fbs-trace

Event-driven temporal weapon sweeps and duplicate-hit tracking.

Requires CMake 3.16+, a C99 compiler and the platform C library. No dependency
on another FinalBuildSystems module or fbs-core is needed.


```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure --no-tests=error
./build/fbs_trace_example
```

Use `add_subdirectory` or CMake FetchContent and link `fbs::trace`:

```cmake
include(FetchContent)
FetchContent_Declare(fbs_trace
  GIT_REPOSITORY https://github.com/finalbuildgames-com/fbs-trace.git
  GIT_TAG main) # Pin a reviewed commit in production.
FetchContent_MakeAvailable(fbs_trace)
target_link_libraries(your_target PRIVATE fbs::trace)
```

Example: [basic.c](examples/basic.c).
Public API: [include/fbs/trace.h](include/fbs/trace.h).
The private FinalBuildSystems monorepo is the source of truth; releases are
curated snapshots. This repository is private pending the owner's review.

MIT for original contributions; see [THIRD_PARTY_NOTICES](THIRD_PARTY_NOTICES)
for upstream licenses and attribution.
