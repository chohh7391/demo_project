// Copyright 2026 Hyunho Cho
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <cmath>

#include "demo_controller/sine_reference.hpp"

namespace
{

using demo_controller::peak_rate;
using demo_controller::sine_sample;

constexpr double kAmplitude = 0.15;
constexpr double kFrequency = 0.5;
constexpr double kRamp = 2.0;

TEST(SineReference, StartsAtRest)
{
  const auto start = sine_sample(0.0, kAmplitude, kFrequency, kRamp);
  EXPECT_DOUBLE_EQ(start.offset, 0.0);
  EXPECT_DOUBLE_EQ(start.rate, 0.0);
  EXPECT_DOUBLE_EQ(start.acceleration, 0.0);
}

TEST(SineReference, RateAndAccelerationAreTheOffsetsDerivatives)
{
  // Central differences, across the ramp and after it.
  const double h = 1e-5;
  for (double t = 0.05; t < 6.0; t += 0.173) {
    const auto before = sine_sample(t - h, kAmplitude, kFrequency, kRamp);
    const auto now = sine_sample(t, kAmplitude, kFrequency, kRamp);
    const auto after = sine_sample(t + h, kAmplitude, kFrequency, kRamp);
    EXPECT_NEAR(now.rate, (after.offset - before.offset) / (2.0 * h), 1e-6) << "t " << t;
    EXPECT_NEAR(now.acceleration, (after.rate - before.rate) / (2.0 * h), 1e-4) << "t " << t;
  }
}

TEST(SineReference, ReachesTheAmplitudeOnceTheRampIsOver)
{
  // A quarter period after the ramp, at a crest.
  const double crest = kRamp + 0.25 / kFrequency;
  EXPECT_NEAR(sine_sample(crest, kAmplitude, kFrequency, kRamp).offset, kAmplitude, 1e-12);
  double largest = 0.0;
  for (double t = 0.0; t < 10.0; t += 1e-3) {
    largest = std::max(largest, std::abs(sine_sample(t, kAmplitude, kFrequency, kRamp).offset));
  }
  EXPECT_LE(largest, kAmplitude + 1e-12);
}

TEST(SineReference, PeakRateIsAmplitudeTimesAngularFrequency)
{
  EXPECT_DOUBLE_EQ(peak_rate(kAmplitude, kFrequency), kAmplitude * 2.0 * M_PI * kFrequency);
  double largest = 0.0;
  for (double t = kRamp; t < 10.0; t += 1e-4) {
    largest = std::max(largest, std::abs(sine_sample(t, kAmplitude, kFrequency, kRamp).rate));
  }
  EXPECT_NEAR(largest, peak_rate(kAmplitude, kFrequency), 1e-6);
}

}  // namespace
