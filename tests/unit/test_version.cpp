#include "tinyspatial/version.hpp"

#include <gtest/gtest.h>

namespace {

TEST(Version, StringMatchesComponents) {
  EXPECT_EQ(tinyspatial::kVersionMajor, 0);
  EXPECT_EQ(tinyspatial::kVersionMinor, 1);
  EXPECT_EQ(tinyspatial::kVersionPatch, 0);
  EXPECT_EQ(tinyspatial::kVersion, "0.1.0");
}

}  // namespace
