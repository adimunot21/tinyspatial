# Set up your machine

This page gets you from "fresh computer" to "I just built and tested a real C++
library." We assume **nothing** — if you've never opened a terminal, you're in
the right place. Take it slowly; you only do this once.

> **Two paths.** You can install the tools directly (Path A), or use Docker,
> which wraps the whole toolchain in a box so nothing touches the rest of your
> system (Path B). If you're unsure, start with Path A on Linux; pick Docker if
> you're on Windows or want maximum isolation.

## What's a terminal?

A **terminal** is a window where you type commands instead of clicking buttons.
A command is just a line of text you type and then press Enter. When this page
shows a box like this:

```bash
echo hello
```

…it means: type `echo hello` into the terminal and press Enter. (`echo` just
prints back what you give it. Try it — it should print `hello`.)

- **Linux:** press `Ctrl`+`Alt`+`T`, or search your apps for "Terminal".
- **macOS:** press `Cmd`+`Space`, type "Terminal", press Enter.
- **Windows:** we strongly recommend Docker (Path B) or WSL2; a raw Windows
  terminal needs extra setup we don't cover here.

## Path A — install the tools directly (Linux)

You need exactly three things: a C++ compiler, the CMake build tool, and Git.
On Ubuntu/Debian, one command installs all three:

```bash
sudo apt update
sudo apt install -y build-essential cmake git
```

`sudo` means "do this as administrator"; it will ask for your password. Nothing
will appear as you type the password — that's normal. Press Enter when done.

Check it worked:

```bash
g++ --version      # should print a version number, 12 or higher
cmake --version    # should print 3.20 or higher
git --version
```

If each prints a version, you're ready. Skip to **Get the code**.

## Path B — use Docker (any OS)

[Install Docker Desktop](https://docs.docker.com/get-docker/) for your operating
system, then start it. You don't need a compiler or CMake — the Docker image
brings its own. Verify Docker is running:

```bash
docker --version
```

## Get the code

"Cloning" means downloading a copy of the project, including its history.

```bash
git clone https://github.com/adimunot21/tinyspatial.git
cd tinyspatial
```

`cd` means "change directory" — you're now *inside* the project folder. This
project leans on a few well-known libraries (for math, for testing). They're
included as **submodules**, which you fetch once with:

```bash
git submodule update --init --recursive
```

This downloads Eigen, GoogleTest, and a few others into `third_party/`. It can
take a minute.

## Build and test it

### If you took Path A (direct install)

```bash
cmake --preset=debug          # set up the build (one time per change to config)
cmake --build build/debug -j  # actually compile; -j uses all your CPU cores
ctest --preset=debug --output-on-failure   # run the tests
```

You should see something ending in:

```
100% tests passed, 0 tests failed out of 1
```

That one passing test checks the library's version string — tiny on purpose.
It proves your whole toolchain works end to end. Everything else in this course
builds on top of this exact loop: **configure → build → test.**

### If you took Path B (Docker)

```bash
docker build --target builder -t tinyspatial:builder .
```

Docker will configure, build, and run the tests inside the container. If it
finishes without an error, you're good.

## When something goes wrong

- **`command not found`** — the tool isn't installed (or the terminal needs
  restarting after install). Re-check the install step.
- **`cmake: version too old`** — you need CMake 3.20+. On older Ubuntu, install
  it from [Kitware's APT repository](https://apt.kitware.com/) or use Docker.
- **Submodule folders are empty** — you forgot
  `git submodule update --init --recursive`. Run it now.
- **Still stuck?** Open an issue on the repository. Paste the *exact* command
  you ran and the *exact* error. "It didn't work" is hard to help with; the
  error text is gold.

## You're set up

You now have a working build of a real robotics library and a green test. That's
genuinely the hardest part of getting started — everything from here is ideas,
not installation.

Next: head into [Chapter 03 — Rotations and transforms](../03_rotations_and_transforms/README.md)
once it's published, or revisit [What is a robot?](01_what_is_a_robot.md).

---

### Where this lives in the library

| Concept | Where it lives |
| ------- | -------------- |
| Build configuration | [`CMakeLists.txt`](../../CMakeLists.txt), [`CMakePresets.json`](../../CMakePresets.json) |
| The test you just ran | [`tests/unit/test_version.cpp`](../../tests/unit/test_version.cpp) |
| Vendored dependencies | [`third_party/`](../../third_party) |
| Docker setup | [`docker/`](../../docker) |
