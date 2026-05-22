# Chapter 0 — Welcome

This short chapter answers three questions before you dive in:

1. What is this thing?
2. Who is it for?
3. What will you actually learn?

## What is this?

`tinyspatial` is a small software library that does the math a robot needs to
move its arm: where is the hand right now, how fast is it moving, what torques
do the motors need, and — going the other way — what joint angles put the hand
*there*. It's written in a programming language called C++, and it's deliberately
small enough that one motivated person can read all of it.

This **course** is a guided tour of that math and that code. Think of the
library as a working engine and the course as the manual that explains every
part — except we also explain the physics of *why* engines work at all.

## Who is it for?

Three kinds of reader, and you might be more than one:

- **The total beginner.** You've maybe never written a line of code. That's
  fine. We start from "what is a robot" and link you to the best free resources
  for the prerequisites (basic programming, basic math) instead of rushing you.
- **The engineer learning robotics.** You can code, but spatial algebra and
  Featherstone's algorithms are new. The chapters from 03 onward are written for
  you: worked examples, no jargon dumps.
- **The reviewer.** You know all this and just want to judge the code. You can
  skip the course entirely and read [`../include/`](../include) — but the
  "Where this lives in the library" tables at the end of each chapter are a fast
  index into the source.

## What will you learn?

By the end you'll understand, and be able to modify, the core of a real
rigid-body dynamics library. Concretely, you'll be able to explain to someone
else:

- why we represent a rotation four different ways and when to use each;
- what a "twist" is and why robotics packs rotation and translation into one
  6-vector;
- how a chain of joints turns into the position of a gripper;
- how the same structure, run backwards, tells you the motor torques;
- and how we *prove* our answers are right by checking them against Pinocchio.

## How to read this course

- **Work the examples by hand.** Robotics math sticks when you compute a small
  case yourself, not when you watch someone else do it.
- **Don't skip the prerequisites you don't have.** Chapters 01 and 02 point you
  to excellent free material. Come back when you're ready.
- **Keep the source open.** Every chapter ends with exact file-and-line links.

Next: [What is a robot?](01_what_is_a_robot.md)

---

### Where this lives in the library

This is a welcome chapter, so there's no algebra to point at yet — but here's
the lay of the land you'll be exploring:

| Concept | Where it lives |
| ------- | -------------- |
| Library version / entry point | [`include/tinyspatial/version.hpp`](../../include/tinyspatial/version.hpp) |
| The roadmap (what's built when) | [`PROJECT_PLAN.md`](../../PROJECT_PLAN.md) |
| How the project is built | [`CMakeLists.txt`](../../CMakeLists.txt) |
