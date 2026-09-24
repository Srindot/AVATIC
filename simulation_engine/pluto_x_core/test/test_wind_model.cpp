// Copyright 2026 AVATIC contributors.

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "pluto_x/environment/wind_model.hpp"
#include "test_helpers.hpp"

namespace pluto_x {
namespace {

WindParams Gusty() {
  WindParams p;
  p.enabled = true;
  p.seed = 7;
  p.mean_enu_m_s = Vector3(1.0, -0.5, 0.0);
  p.gust_std_m_s = 0.3;
  p.gust_time_constant_s = 2.0;
  return p;
}

TEST(WindModel, DisabledIsCalm) {
  WindModel wind(test::LoadDefaultConfig().wind);
  EXPECT_TRUE(wind.Step(0.001).isZero());
}

TEST(WindModel, DeterministicForSeed) {
  WindModel a(Gusty());
  WindModel b(Gusty());
  for (int i = 0; i < 1000; ++i) {
    ASSERT_EQ(a.Step(0.001), b.Step(0.001));
  }
}

TEST(WindModel, MeanAndGustStatistics) {
  WindModel wind(Gusty());
  // 2000 s of 10 ms steps = 1000 correlation times: sample std within ~5 %.
  const int steps = 200000;
  Vector3 sum = Vector3::Zero();
  Vector3 sum_sq = Vector3::Zero();
  for (int i = 0; i < steps; ++i) {
    const Vector3 gust = wind.Step(0.01) - Gusty().mean_enu_m_s;
    sum += gust;
    sum_sq += gust.cwiseProduct(gust);
  }
  const Vector3 mean = sum / steps;
  for (int axis = 0; axis < 3; ++axis) {
    const double std_dev = std::sqrt(sum_sq[axis] / steps - mean[axis] * mean[axis]);
    EXPECT_NEAR(std_dev, 0.3, 0.3 * 0.1) << "axis " << axis;
    EXPECT_NEAR(mean[axis], 0.0, 0.05) << "axis " << axis;
  }
}

TEST(WindModel, RejectsInvalidParameters) {
  WindParams p = Gusty();
  p.gust_time_constant_s = 0.0;
  EXPECT_THROW(WindModel{p}, std::invalid_argument);
  p = Gusty();
  p.gust_std_m_s = -1.0;
  EXPECT_THROW(WindModel{p}, std::invalid_argument);
}

}  // namespace
}  // namespace pluto_x
