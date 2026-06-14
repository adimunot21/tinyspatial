# Developing inside Docker

You don't *need* Docker to work on `tinyspatial` — a host with
`build-essential cmake git` is enough. Docker is here for two reasons:

1. A reproducible build that matches CI exactly.
2. An isolated home for the **Pinocchio validation oracle**, so conda's
   libstdc++ never leaks into the standalone C++ build.

## Stages

The [`Dockerfile`](Dockerfile) is multi-stage:

| Stage               | What it is                                                            |
| ------------------- | --------------------------------------------------------------------- |
| `builder`           | Full toolchain. Configures, builds, and runs the unit tests.          |
| `runtime`           | Minimal image carrying headers + license (future: the Python wheel).  |
| `validation-oracle` | **Phase 0 stub.** Becomes the conda-forge Pinocchio 3.9 parity env.   |

## Common commands

```bash
# Build and test the library exactly as CI does:
docker build --target builder -t tinyspatial:builder .

# Or via compose, from the docker/ directory:
docker compose build builder
```

> The `validation-oracle` stage is intentionally incomplete until Phase 4,
> when the Pinocchio parity suite lands.
