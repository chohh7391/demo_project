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

#pragma once

#include <algorithm>
#include <cmath>

namespace demo_controller
{

// One sample of the swept joint's motion, relative to where it was activated.
struct SineSample
{
  double offset{0.0};        // [rad]
  double rate{0.0};          // [rad/s]
  double acceleration{0.0};  // [rad/s^2]
};

// The offset a(t) sin(2 pi f t), its rate and its acceleration, t seconds after
// activation. The amplitude a(t) rises from 0 to `amplitude` over `ramp_sec`
// along a smoothstep, 3x^2 - 2x^3 of x = t / ramp_sec, whose value and slope are
// 0 at the start: the joint leaves its position at rest, with no step in
// position, rate or acceleration. No ROS, so the test drives it directly.
inline SineSample sine_sample(double t, double amplitude, double frequency, double ramp_sec)
{
  const double w = 2.0 * M_PI * frequency;
  const double x = std::clamp(t / ramp_sec, 0.0, 1.0);
  const bool ramping = t < ramp_sec;
  const double a = amplitude * x * x * (3.0 - 2.0 * x);
  const double da = ramping ? amplitude * 6.0 * x * (1.0 - x) / ramp_sec : 0.0;
  const double dda = ramping ? amplitude * (6.0 - 12.0 * x) / (ramp_sec * ramp_sec) : 0.0;
  const double s = std::sin(w * t);
  const double c = std::cos(w * t);
  SineSample sample;
  sample.offset = a * s;
  sample.rate = da * s + a * w * c;
  sample.acceleration = dda * s + 2.0 * da * w * c - a * w * w * s;
  return sample;
}

// The largest rate the sweep reaches, once the ramp is over: amplitude * 2 pi f.
inline double peak_rate(double amplitude, double frequency)
{
  return amplitude * 2.0 * M_PI * frequency;
}

}  // namespace demo_controller
