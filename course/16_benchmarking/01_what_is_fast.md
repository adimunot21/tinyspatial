# Latency, throughput, and what 'fast' means

"`tinyspatial`'s RNEA runs in 4.5 microseconds per call on a Franka." Is
that fast?

It depends on what the number is used for.

## Three flavours of performance

**Latency** — the wall-clock time for a single call. Matters when:

- The code runs inside a real-time control loop at 1 kHz: 1 ms = 1000
  microseconds is the *entire* budget per cycle. RNEA at 4.5 μs uses 0.5%
  of that.
- A user presses a button and waits for a response.

**Throughput** — calls per second per core when fully pipelined. Matters
when:

- Running 1000 random configurations in a sampling-based planner.
- Batching IK queries.
- Training a model that needs a million forward-dynamics steps per
  epoch.

Latency and throughput are not always the same. If a computation has a
pipeline with N stages each taking T time, *latency* is N·T (one full
pipeline must finish) but *throughput* is 1/T (one stage completes
per cycle once the pipeline is full). For these algorithms, where one call
is one independent computation, the two are identical: throughput is just
1 / latency.

**Asymptotic complexity** — how time grows with input size. For RNEA on
a serial chain with N joints, asymptotic complexity is O(N). The
constant matters most when N is small (typical robot arms have N=6 or 7).

## Why a raw nanosecond number lies

"4500 ns / RNEA call" means nothing without:

- **What machine?** Server-grade Xeon vs laptop i7 vs phone ARM differ by
  3–10×.
- **What compiler flags?** `-O3` vs `-O0` is typically 5–20× difference.
- **What's the rest of the system doing?** Background processes pushing
  the cache around can change a benchmark by 30%.
- **What's the input?** RNEA at `q = 0, v = 0, a = 0` short-circuits a
  lot of the inner work (zero × matrix = skip). Random configurations
  trigger the full code path. The library's benchmarks deliberately use
  random inputs from a fixed seed.
- **Is the cache warm?** Cold-cache first call vs hot-cache N+1 call can
  easily differ 2×.

This is why the numbers in `docs/BENCHMARKS.md` come with a full machine
description and reproducible commands. *Different machines produce
different numbers.* What stays stable is the *ratio* to a
reference implementation — here, Pinocchio. The ratio holds across
machines much better than the absolute number.

## What 'within 1.4× of Pinocchio' means

The performance bar is **RNEA throughput on a 7-DoF Franka, within
1.4× of Pinocchio**. Two reasons that ratio is the right metric:

1. **Pinocchio is the reference implementation the field
   benchmarks against.** A credible claim of parity ± 40%
   demonstrates that the gap isn't algorithmic ignorance — just the
   normal "first version of any library is a little slower than the
   third version of the most-optimised one" delta.
2. **It's reproducible.** Pinocchio is open source; anyone can re-run
   the comparison. It's portable to whatever hardware they have.

What ratios mean in practice on Franka RNEA (per `docs/BENCHMARKS.md`):

| Ratio | Reading                                                           |
| ----: | ----------------------------------------------------------------- |
|  1.0× | Identical performance.                                            |
|  1.4× | **The bar.** "Comparable to Pinocchio."                           |
|  2.0× | Slower but credible. Most amateur libraries land here.            |
|  3.0× | Where we are now in pure C++. Architectural gap, not just tuning. |
|  9.0× | Where we are from Python — most of that is binding overhead.      |
|  30×+ | Something is fundamentally broken; revisit the algorithm.         |

## What we measure, what we ignore

In `tinyspatial`'s benchmarks we **measure**:

- Time per call to a single algorithm (RNEA, ABA, CRBA, FK, Jacobian, IK).
- Each of the four fixture URDFs (`simple_arm`, `franka_fr3`, `ur5e`,
  `so_arm101`).
- Both C++ direct and Python-via-binding.

We **deliberately don't measure**:

- The time to *load* the URDF (one-time cost, dominated by file I/O).
- Memory consumption (matters for embedded use cases; not our focus yet).
- Power consumption (likewise).
- Multi-threaded throughput (the algorithms are intrinsically serial per
  call; parallelism would mean batching multiple `(q,v,a)` triples).

Every benchmark invites one question: *what does this number actually
indicate?* A number without context is just a digit.

## The framing for the rest of this chapter

The remaining chapters:

1. Walk through Google Benchmark's API (next chapter).
2. Use `perf` and callgrind to find *where* the time goes (chapter 03).
3. Apply a real optimisation, measure the delta, and decide whether it was
   worth it (chapter 04).

That's the whole loop. Done well, it's the engine that turns "this should
be faster" into "this is now 1.3× faster, and here's the commit
that proves it."
