# tinyspatial multi-stage Dockerfile.
#
# Stages:
#   builder           - full toolchain, builds + tests the library.
#   runtime           - minimal image carrying just the built artifacts.
#   validation-oracle - STUB for Phase 0; later wires in conda-forge Pinocchio
#                       3.9 so we can run the parity suite in an isolated env
#                       that never contaminates the standalone C++ build.
#
# Build a specific stage with:  docker build --target builder -t tinyspatial:builder .

# ---------------------------------------------------------------------------
# Stage 1: builder
# ---------------------------------------------------------------------------
FROM ubuntu:22.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      g++-12 \
      cmake \
      git \
      ca-certificates \
    && rm -rf /var/lib/apt/lists/*

ENV CC=gcc-12 CXX=g++-12

WORKDIR /src
COPY . .
RUN git submodule update --init --recursive || true \
    && cmake --preset=debug \
    && cmake --build build/debug -j \
    && ctest --preset=debug --output-on-failure

# ---------------------------------------------------------------------------
# Stage 2: runtime (minimal; the library is header-mostly, so this is mostly a
# placeholder for future compiled artifacts such as the Python wheel)
# ---------------------------------------------------------------------------
FROM ubuntu:22.04 AS runtime

RUN apt-get update && apt-get install -y --no-install-recommends \
      libstdc++6 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/tinyspatial
COPY --from=builder /src/include ./include
COPY --from=builder /src/LICENSE ./LICENSE

# ---------------------------------------------------------------------------
# Stage 3: validation-oracle (STUB — fleshed out in Phase 4 when the Pinocchio
# parity suite arrives). Intentionally a SEPARATE image: conda's libstdc++ must
# never leak into the standalone C++ build (CLAUDE.md §15).
# ---------------------------------------------------------------------------
FROM continuumio/miniconda3:latest AS validation-oracle

# TODO(Phase 4): pin conda-forge pinocchio==3.9.0 and copy in tests/validation/.
#   RUN conda install -c conda-forge pinocchio=3.9.0 -y
RUN echo "validation-oracle stage is a Phase 0 stub; see CLAUDE.md §11"

WORKDIR /work
