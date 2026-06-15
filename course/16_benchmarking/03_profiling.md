# Profiling with `perf` and callgrind

A benchmark reports *how much* time was spent. A profiler reports
*where* it was spent. The two together are the optimisation toolkit.

This chapter covers two profilers: `perf` (Linux hardware counter sampling,
low overhead) and Valgrind's `callgrind` (instruction-level instrumentation,
high overhead but exact).

## `perf` — the lightweight profiler

`perf` reads Linux's perf-events subsystem to sample the running program
every few thousand cycles and record where it was. Total overhead: ~1%.

### Five essential `perf` commands

```bash
# 1. Count total instructions, cycles, branch mispredicts, cache misses.
perf stat ./bench_rnea --benchmark_filter=franka

# 2. Sample the program counter and aggregate by function.
perf record ./bench_rnea --benchmark_filter=franka
perf report                       # interactive UI
perf report --stdio | head -40    # text dump

# 3. Annotate the hottest function with source + assembly.
perf annotate <function_name>

# 4. Flamegraph (needs an extra script from Brendan Gregg).
perf record -F 99 -g ./bench_rnea --benchmark_filter=franka
perf script | flamegraph.pl > flame.svg

# 5. The lowest-overhead 'where is the time' check.
perf stat -e task-clock,context-switches,page-faults ./bench_rnea
```

### `perf stat` output, annotated

```
$ perf stat ./bench_rnea --benchmark_filter=franka

 Performance counter stats for './bench_rnea ...':

         705.84 msec task-clock                #    1.000 CPUs utilized
              0      context-switches          #    0.000 /sec
              4      page-faults               #    5.667 /sec
  3,134,628,729      cycles                    #    4.441 GHz
  5,488,213,012      instructions              #    1.75  insn per cycle
    817,254,012      branches                  #    1.158 G/sec
      9,331,082      branch-misses             #    1.14% of all branches
```

What to look at:

- **`insn per cycle` (IPC)** — modern x86 can sustain ~3–4 if the code
  is well-pipelined. 1.75 means we're stalling on something (probably
  cache or branch misses). High IPC = the CPU is busy doing useful work;
  low IPC = it's waiting.
- **`branch-misses`** — every miss costs ~20 cycles. 1.14% is fine. >5%
  is suspicious. Indirect calls through a `std::variant` visitor are a
  common cause.
- **`page-faults`** — should be near zero for a steady-state benchmark.
  Lots of page faults = allocation is happening in the hot loop.

### `perf record` and `perf report`

`perf record` writes a `perf.data` file. `perf report` reads it and
shows a tree of where the samples landed:

```
# Samples: 28K of event 'cycles'
# Event count (approx.): 25,486,109,201
#
# Overhead  Command          Symbol
# ........  ...............  ...........................................
#
    18.42%  bench_rnea       tinyspatial::rnea
    14.21%  bench_rnea       Eigen::internal::scalar_product_op<...>
     9.84%  bench_rnea       std::visit
     7.12%  bench_rnea       tinyspatial::SE3::operator*
     ...
```

Each row states "X% of all CPU cycles were spent inside this
function." Speeding up the function at the top cuts total
time by close to that percentage. (Famously: optimising
something at 0.1% is not worth it — even infinite speedup saves 0.1%.)

### The `perf_event_paranoid` gotcha

On Ubuntu 22+ the default `/proc/sys/kernel/perf_event_paranoid` is
either 3 or 4, both of which block per-process performance counters for
unprivileged users. The symptom:

```
Access to performance monitoring and observability operations is limited.
```

Three options:

```bash
# A. Temporary, until reboot:
sudo sh -c 'echo -1 > /proc/sys/kernel/perf_event_paranoid'

# B. Permanent, via sysctl:
sudo sh -c 'echo "kernel.perf_event_paranoid = 1" > /etc/sysctl.d/99-perf.conf'

# C. Add CAP_PERFMON to your binary (NixOS / containerised setups):
sudo setcap cap_perfmon=ep ./bench_rnea
```

On a machine with sudo access, the temporary `perf_event_paranoid`
setting is the easiest fix when profiling access is needed.

## Callgrind — the slow, exact profiler

`callgrind` is part of Valgrind. It runs the code inside a virtual machine
and records every instruction executed. The overhead is **~20–50×**, but
the data is precise: it reports exact call counts and per-instruction
breakdowns.

```bash
valgrind --tool=callgrind ./bench_rnea --benchmark_filter=franka \
         --benchmark_min_time=0.001s          # short, because it's slow
callgrind_annotate callgrind.out.<pid> | head -50
```

The output:

```
80,000,000  rnea.hpp:75  tinyspatial::rnea(...)
72,000,000  rnea.hpp:82  data.v[i] = ...
64,000,000  motion.hpp:42 Motion::cross(...)
```

Each number is *instructions executed at that line*. Much finer-grained
than `perf record`'s statistical sampling.

The companion GUI is `kcachegrind`:

```bash
kcachegrind callgrind.out.<pid>
```

Beautiful call tree, source/assembly annotation side-by-side, sortable
by self vs cumulative cost. Given an X server (or X forwarding),
this is by far the nicest profiling experience on Linux.

### Why use callgrind when `perf` exists

- **No kernel permission issues.** Callgrind is pure user-mode emulation.
- **No CPU frequency noise.** Every instruction is counted, not sampled.
- **Call counts for free.** "This function was called 100,000 times,
  consuming an average of 800 instructions each."
- **Per-line cost breakdown.** `perf annotate` has this but at sample
  resolution, so small functions get noisy.

### When to use which

| Need                                  | Tool        |
| ------------------------------------- | ----------- |
| Wall-clock breakdown by function      | `perf record` |
| Hardware counter detail (cache, IPC)  | `perf stat`   |
| Per-instruction cost                  | callgrind    |
| Production / always-on monitoring     | `perf` (cheap) |
| Verifying a small loop's inlining     | `perf annotate` |
| Counting how many times X is called   | callgrind    |

For tinyspatial we used a mix: `perf stat` to spot stall patterns, `perf
report` to find the top 5 hot functions, then dropped into callgrind for
the inner loop of RNEA when we wanted to know exactly which lines were
expensive.

## What profiling typically reveals

Real RNEA / FK profiling on the current `tinyspatial` codebase shows:

1. **`std::visit` dispatch on `JointVariant`** — about 8–12% of total
   cycles. The variant visitor compiles to a vtable-equivalent indirect
   call per joint. Replacing with a small switch on a pre-baked discriminator
   integer would shave most of that.
2. **`SO3::act(point)` quaternion-vector product** — quaternions are
   compact (4 doubles vs 9 for a matrix) but acting on a vector costs
   ~30 floating-point ops vs 9 for a matrix multiply. The right tradeoff
   depends on how many acts occur per stored rotation; for FK each
   joint frame is acted on ~5 times per call, so caching a matrix wins.
3. **`Eigen::Quaternion::operator*`** — composing two rotations is
   ~28 ops on quaternions, slightly cheaper than the ~36 ops on
   matrices, so this one stays as is.
4. **Heap allocations from temporary `Eigen::MatrixX`** — `Eigen` is
   smart enough to allocate on the stack for fixed-size types. Code
   that uses dynamic-size types (`MatrixX`, `VectorX`) in inner loops
   pays a malloc tax. Look for these and convert to fixed-size where
   possible.

The next chapter walks through fixing one of these and measuring the
result, end-to-end.
