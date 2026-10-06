# fbs-trace

Swept melee weapon traces against moving spheres and capsules, with per-attack duplicate-hit tracking, in C99.

## What it does

A fast sword swing can pass through a thin target between two frames. This library tests the whole motion of the weapon from its previous pose to its current pose, and reports an estimated contact time inside the frame under the selected sweep policy.

- The weapon is a segment from `a` (hilt) to `b` (tip) with an optional radius. `a == b` is a point weapon such as a fist.
- `fbs_trace_sweep_sphere` and `fbs_trace_sweep_capsule` sweep a weapon against one sphere or capsule that may also move. On contact they return `FBS_TRACE_CONTACT` and fill an `fbs_trace_contact`: the normalized contact time in the interval, the contact point on the weapon, the closest point on the target, a normal, and the motion of the struck weapon point (absolute and relative to the target). `fbs_trace_closest` does the same test for a single static pose.
- A trace context (`fbs_trace_create`) tracks attacks per weapon: `fbs_trace_begin`, `fbs_trace_update` once per frame with the new pose and time, `fbs_trace_end`. Optional time windows limit when the attack is active. `fbs_trace_break` drops the stored samples after a teleport or animation snap so no sweep spans the jump.
- `fbs_trace_test` sweeps the current interval against a list of targets (each with a target id and a body zone id) and returns the hits sorted by contact time, ties kept in input order.
- Duplicate-hit policy per context: report a target once per attack, once per (target, zone) pair, every time, or only what you mark yourself with `fbs_trace_mark`.
- Two sweep policies. `fbs_trace_policy_default()` uses conservative advancement (tolerance 1e-4, at most 64 iterations). `fbs_trace_policy_compat_4cm()` samples the motion uniformly with at most 0.04 units of travel between samples (at most 1024 samples).
- If your engine runs its own shape casts, `fbs_trace_substeps` writes the interpolated weapon poses for the current interval instead.

## When to use it

- Melee combat where weapon hits come from animated poses and frame rates vary. `tests/test_trace.c` (`test_frame_rate_schedules`) drives the same swing at 30, 60 and 144 Hz and gets exactly one hit per swing each time; with the default policy the hit time is within 1e-3 s of the analytic 0.3 s.
- Hitboxes that are spheres or capsules (heads, limbs, torsos) and that may move during the frame.
- Games that need one hit per body zone per attack, or several weapons swinging at once (`test_dual_weapons` keeps samples and duplicate memory separate per weapon).

## When not to use it

- Target shapes are spheres and capsules only. There are no boxes, meshes or terrain.
- There is no broad phase and no scene query. You pass the candidate targets to every `fbs_trace_test` call and they are tested one by one.
- Capacities are fixed when the context is created: concurrent weapon sessions (`max_weapons`, default 4), windows per attack (`max_windows`, default 8) and remembered hits per attack (`max_struck`, default 64). Each is limited to 1..1048576. A full table returns an error instead of growing.
- Weapon endpoints move in a straight line between two samples. A wide arc between two frames is treated as its chord, so feed poses often enough for the arc you need.
- The combined radius comes from the start of each interval and is not interpolated across it.
- The sampled policy can step over targets thinner than its spacing. `test_thin_target_fast_sweep` shows the 4 cm preset missing a sphere of radius 0.019 that the default policy hits.
- The default policy reports contact once the gap is within its tolerance, so the contact can come slightly before the surfaces touch.
- `FBS_TRACE_E_CAPACITY` means a sweep needs more samples/iterations than `policy.max_steps` allows (or a context has no free weapon slot). Handle that outcome explicitly; it is not a no-contact result.
- It finds contacts only. Choosing which hit deals damage is up to your game code.

## Example

A blade sweeps along +x at 3 units per second, updated at 30 Hz, past a sphere of radius 0.1 at the origin. This follows `run_schedule` in `tests/test_trace.c`.

```c
#include <fbs/trace.h>
#include <stdio.h>

int main(void) {
  fbs_trace_config config = fbs_trace_config_default(); /* default policy, per-zone dedup */
  fbs_trace_context *trace = NULL;
  fbs_trace_target head = {0};
  int frame, failed = 0;

  head.target_id = 7u;                     /* the victim */
  head.zone_id = 0u;                       /* its body zone */
  head.kind = FBS_TRACE_TARGET_SPHERE;     /* sphere: current.a == current.b, the centre */
  head.current.radius = 0.1f;              /* has_previous = 0: the target stands still */

  if (fbs_trace_create(&config, NULL, &trace) != FBS_TRACE_OK) return 1;
  if (fbs_trace_begin(trace, 1u, 42u, NULL, 0) != FBS_TRACE_OK) failed = 1;

  for (frame = 0; !failed && frame <= 21; ++frame) {
    double time = frame / 30.0;
    float x = (float)(-1.0 + 3.0 * time);
    fbs_trace_weapon blade = {{x, -0.5f, 0.0f}, {x, 0.5f, 0.0f}, 0.0f};
    fbs_trace_hit hits[4];
    fbs_trace_interval interval;
    size_t count = 0, i;

    if (fbs_trace_update(trace, 1u, 42u, (float)time, &blade) != FBS_TRACE_OK ||
        fbs_trace_test(trace, 1u, &head, 1u, hits, 4u, &count) != FBS_TRACE_OK ||
        fbs_trace_interval_get(trace, 1u, &interval) != FBS_TRACE_OK) {
      failed = 1;
      break;
    }
    for (i = 0; i < count; ++i) {
      double at = interval.previous_time +
                  hits[i].contact.time * (interval.current_time - interval.previous_time);
      printf("hit target %u zone %u at %.3f s, blade x = %.3f\n", hits[i].target_id,
             hits[i].zone_id, at, hits[i].contact.point.x);
    }
  }
  fbs_trace_destroy(trace);
  return failed;
}
```

Build it as any executable that links `fbs::trace` (for example `add_executable(swing swing.c)` and `target_link_libraries(swing PRIVATE fbs::trace)`). It prints one hit at about 0.300 s.

## Build and test

Run from this repository's root. In addition to CMake and the compiler named
below, install the build tool selected by your generator (for example Make or
Ninja).

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build --parallel 1
(cd build && ctest --output-on-failure)
```

This runs two tests. `trace` runs `tests/test_trace.c`: hand-derived sweep cases (thin targets, tip and middle contact, a rotating blade, point weapons, moving targets, parallel and collinear axes), frame-rate schedules, the session lifecycle, dual weapons, hit ordering and truncation, all four duplicate policies, malformed input (every rejected call must leave its output untouched), capacity and allocator failures, and a replay of the 34 recorded cases in `tests/fixtures/trace/sweeps.json`, where the status must match exactly and every contact value within 1e-6. It also prints sweep timings, which are not checked. `trace_example` runs `examples/basic.c` (`fbs_trace_example`), which creates and destroys a context and prints the API version.

Requirements: CMake 3.16 or newer and a C99 compiler. The library links `libm` on non-MSVC toolchains. There is no vendored third-party code. Options: `FBS_BUILD_TESTS` (needs `BUILD_TESTING`) and `FBS_BUILD_EXAMPLES`, both on by default.

Use it with `add_subdirectory` or FetchContent and link `fbs::trace`. The repository does not install a CMake package config file, so `find_package` is not supported.

```cmake
include(FetchContent)
set(FBS_BUILD_TESTS OFF)
set(FBS_BUILD_EXAMPLES OFF)
FetchContent_Declare(fbs_trace
  GIT_REPOSITORY https://github.com/finalbuildgames-com/fbs-trace.git
  GIT_TAG <full commit hash>) # pin a reviewed commit
FetchContent_MakeAvailable(fbs_trace)
target_link_libraries(your_target PRIVATE fbs::trace)
```

This repository ships the C library only. Engine bindings and adapters are not included.

## Build modes and installation

`BUILD_SHARED_LIBS=ON` builds a shared library; the default is static.
`FBS_BUILD_TESTS` and `BUILD_TESTING` together enable the core test.
`FBS_BUILD_EXAMPLES` controls `fbs_trace_example`; its CTest entry also requires
`BUILD_TESTING`. For a library-only build, set `FBS_BUILD_TESTS=OFF` and
`FBS_BUILD_EXAMPLES=OFF`.

```sh
cmake --install build --prefix "$PWD/install"
```

Installation supplies [the public header](include/fbs/trace.h), the library,
license notices and `FinalBuildTraceTargets.cmake` under
`${CMAKE_INSTALL_LIBDIR}/cmake/FinalBuildTrace`. It supplies no package config or
version config, so `find_package(FinalBuildTrace)` is unavailable. A consumer may
include the installed targets file explicitly and link `fbs::trace`, or use
the source integration above. The [minimal program](examples/basic.c) and
[core tests](tests/test_trace.c) show the implemented entry points.

## Design notes

- Determinism: no globals, no static mutable state and no randomness. The API takes `float`, but all math runs in `double` and is converted once on output. The generated CMake compiles with `-ffp-contract=off` on every non-MSVC toolchain so fused multiply-add does not change results. Hits come back sorted by time with ties in input order.
- Memory: `fbs_trace_create` makes exactly one allocation sized from the config, and `fbs_trace_destroy` frees it. `test_capacity_and_allocator` checks that a whole begin, update and test cycle allocates nothing more. Pass an `fbs_trace_allocator` (alloc, free, user pointer) to replace `malloc`/`free`. `fbs_trace_memory` reports the block size. The sweep functions do not allocate.
- Units: distances are in your units and window times in your time unit. Nothing depends on an up axis or handedness.
- Threading: different contexts can be used from different threads at the same time. One context must not be used from two threads at once.
- Errors: functions return an `fbs_trace_status`. Negative values are errors, and on error nothing is written except where the header says otherwise (`FBS_TRACE_E_FULL` and `FBS_TRACE_E_DEDUP` can write partial results; `fbs_trace_test` and `fbs_trace_substeps` set `*count`, which for invalid targets is the index of the bad one). NaN or infinite coordinates and negative radii return `FBS_TRACE_E_INVALID`. `fbs_trace_status_name` gives a printable name.
- Versioning: the header is version 0.1.0 (`FBS_TRACE_VERSION` is 100, packed as major * 10000 + minor * 100 + patch). `fbs_trace_version()` returns the compiled value so callers can check it at runtime. Declarations are wrapped in `extern "C"` for C++.

## License

MIT for Final Build Games' original code, see [LICENSE](LICENSE). The segment closest-point routine follows the algorithm in Christer Ericson's Real-Time Collision Detection, section 5.1.9; the implementation is original C and no book text is included. See [THIRD_PARTY_NOTICES](THIRD_PARTY_NOTICES).
