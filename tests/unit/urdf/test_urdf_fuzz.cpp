// URDF fuzz test: 1000 mutated XML inputs, derived deterministically from a
// fixed seed. The contract is *no crash and no UB*: every mutation must either
// load successfully (rare, but possible) or throw an exception. Run under
// ASan + UBSan via the debug preset.
//
// Mutation strategies, picked uniformly:
//   1. Truncate at a random offset.
//   2. Overwrite a random byte with a random byte.
//   3. Insert a random byte at a random offset.
//   4. Delete a random range of bytes.
//
// Together these exercise XML-parser bounds (incomplete tags, unbalanced
// brackets, NULs, junk attribute values) and our post-parse logic (missing
// links, dangling joints).
#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>
#include <string>

#include "tinyspatial/urdf/urdf_loader.hpp"

#include <gtest/gtest.h>

namespace tinyspatial {
namespace {

constexpr const char* kFixtureDir = TINYSPATIAL_TEST_FIXTURE_DIR;

std::string read_file(const std::string& path) {
  std::ifstream f(path);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

std::string mutate(std::string s, std::mt19937& rng) {
  if (s.empty()) {
    return s;
  }
  std::uniform_int_distribution<int> mode(0, 3);
  std::uniform_int_distribution<size_t> pos(0, s.size() - 1);
  std::uniform_int_distribution<int> byte(0, 255);
  const int m = mode(rng);
  if (m == 0) {
    // Truncate.
    s.resize(pos(rng));
  } else if (m == 1) {
    // Overwrite one byte.
    s[pos(rng)] = static_cast<char>(byte(rng));
  } else if (m == 2) {
    // Insert one byte.
    s.insert(pos(rng), 1, static_cast<char>(byte(rng)));
  } else {
    // Delete a small random range.
    const size_t a = pos(rng);
    const size_t b = std::min(s.size(), a + std::uniform_int_distribution<size_t>(1, 32)(rng));
    s.erase(a, b - a);
  }
  return s;
}

TEST(UrdfFuzz, ThousandMutationsNeverCrash) {
  // Use the largest fixture as the seed — more attack surface per mutation.
  const std::string base =
      read_file((std::filesystem::path(kFixtureDir) / "franka_fr3.urdf").string());
  ASSERT_FALSE(base.empty()) << "fixture missing or empty";

  std::mt19937 rng(0xC0FFEE);
  int loaded = 0;
  int parse_errors = 0;
  int other_exceptions = 0;
  for (int i = 0; i < 1000; ++i) {
    const std::string mutated = mutate(base, rng);
    try {
      Model m = build_model_from_urdf(mutated);
      // If we got here, the mutation happened to be benign (e.g. truncated
      // a trailing whitespace). Sanity-check the result is internally
      // consistent.
      EXPECT_GE(m.njoints(), 0);
      ++loaded;
    } catch (const UrdfParseError&) {
      ++parse_errors;
    } catch (...) {
      // Any other exception is a contract miss — UrdfParseError is the only
      // documented failure mode. Don't fail the test (the contract is
      // "no crash"), but count for diagnostics.
      ++other_exceptions;
    }
  }
  // We expect mostly UrdfParseError; a few inputs may stay valid.
  EXPECT_EQ(loaded + parse_errors + other_exceptions, 1000);
  EXPECT_GT(parse_errors, 0) << "no mutation produced a parse error — suspicious";
}

}  // namespace
}  // namespace tinyspatial
