#include "pi_arm_hardware/motion_detector.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <limits>

namespace
{
constexpr double rad = 3.14159265358979323846 / 180.0;
constexpr double threshold = 0.01 * rad;

class Trace
{
public:
  explicit Trace(double ratio = 6.0) : ratio_(ratio) {}
  bool tick(double output_deg, double protocol_speed = 0.0, bool fresh = true)
  {
    time_ += 0.02;
    return detector.update(output_deg * rad, time_, protocol_speed / ratio_ * rad,
      time_, time_, fresh, 0.02 / ratio_ * rad, threshold);
  }
  void settle()
  {
    for (int i = 0; i < 50; ++i) {tick(0.0);}
    ASSERT_FALSE(tick(0.0));
  }
  pi_arm_hardware::MotionDetector detector;
  double time_{1.0};
  double ratio_;
};

TEST(MotionDetector, StaticEncoderJitterAndIsolatedSpeedSpikesDoNotFlicker)
{
  for (double ratio : {6.0, 36.0}) {
    Trace trace(ratio);
    trace.settle();
    for (int i = 0; i < 1000; ++i) {
      // Protocol position fluctuates by two ticks; raw speed spikes to +/-5.
      const double angle = (i % 3 - 1) * 0.01 / ratio;
      const double speed = i % 23 == 0 ? 5.0 : (i % 29 == 0 ? -5.0 : 0.0);
      EXPECT_FALSE(trace.tick(angle, speed)) << "ratio=" << ratio << " frame=" << i;
    }
  }
}

TEST(MotionDetector, DetectsLowSpeedEvenWhenProtocolSpeedRoundsToZero)
{
  for (double ratio : {6.0, 36.0}) {
    for (double direction : {-1.0, 1.0}) {
      Trace trace(ratio);
      trace.settle();
      for (int i = 1; i <= 100; ++i) {
        // 0.02 output degrees/s, sampled at the protocol position resolution.
        const double angle = std::round(direction * 0.02 * i * 0.02 * ratio * 100) /
          (ratio * 100);
        const bool moving = trace.tick(angle);
        if (i >= 30) {EXPECT_TRUE(moving) << "frame=" << i;}
      }
    }
  }
}

TEST(MotionDetector, SustainedSpeedWithFrozenAngleRemainsConservativelyMoving)
{
  Trace trace;
  trace.settle();
  for (int i = 0; i < 100; ++i) {
    const bool moving = trace.tick(0.0, 1.0);
    if (i >= 8) {EXPECT_TRUE(moving);}
  }
  for (int i = 0; i < 10; ++i) {EXPECT_TRUE(trace.tick(0.0));}
  for (int i = 0; i < 20; ++i) {trace.tick(0.0);}
  EXPECT_FALSE(trace.tick(0.0));
}

TEST(MotionDetector, PositionMotionDetectedAndSettlesWithoutResidualSpeedFlicker)
{
  Trace trace;
  trace.settle();
  for (int i = 1; i <= 50; ++i) {
    const bool moving = trace.tick(i * 0.02 * 5.0, 30.0);
    if (i >= 4) {EXPECT_TRUE(moving);}
  }
  for (int i = 0; i < 100; ++i) {
    const bool moving = trace.tick(5.0, i % 9 == 0 ? 2.0 : 0.0);
    if (i >= 50) {EXPECT_FALSE(moving);}
  }
}

TEST(MotionDetector, CachedSamplesCannotConfirmMotionOrStationary)
{
  Trace trace;
  trace.settle();
  const auto t = trace.time_;
  EXPECT_FALSE(trace.detector.update(0, t + 0.02, rad, t + 0.02,
      t + 0.02, true, 0.003 * rad, threshold));
  for (int i = 3; i < 20; ++i) {
    EXPECT_FALSE(trace.detector.update(0, t + 0.02, rad, t + 0.02,
        t + i * 0.01, true, 0.003 * rad, threshold));
  }
  // Even before the ordinary 1-second feedback timeout, a long gap makes
  // stillness unproven. Reconnection requires fresh settling observations.
  EXPECT_TRUE(trace.detector.update(0, t + 0.02, 0, t + 0.02,
      t + 0.5, true, 0.003 * rad, threshold));
  EXPECT_TRUE(trace.detector.update(0, t + 0.6, 0, t + 0.6,
      t + 0.6, true, 0.003 * rad, threshold));
  EXPECT_TRUE(trace.detector.update(0, t + 0.6, 0, t + 0.6,
      t + 0.8, true, 0.003 * rad, threshold));
}

TEST(MotionDetector, InvalidFeedbackAndBackwardTimestampCannotProveStillness)
{
  Trace trace;
  trace.settle();
  EXPECT_TRUE(trace.tick(0.0, 0.0, false));
  EXPECT_TRUE(trace.tick(0.0));
  EXPECT_TRUE(trace.tick(std::numeric_limits<double>::quiet_NaN()));
  trace.settle();
  EXPECT_TRUE(trace.detector.update(0, 0.1, 0, 0.1, 0.1, true, 0.003 * rad, threshold));
}

TEST(MotionDetector, OnePositionOutlierDoesNotLatchMotion)
{
  Trace trace;
  trace.settle();
  EXPECT_FALSE(trace.tick(0.5));
  for (int i = 0; i < 100; ++i) {EXPECT_FALSE(trace.tick(0.0));}
}

TEST(MotionDetector, OscillationWithWindowPeriodIsStillMotion)
{
  Trace trace;
  trace.settle();
  for (int i = 0; i < 200; ++i) {
    const double angle = std::round(0.02 * std::sin(i * 0.02 * 2 * 3.141592653589793 / 0.5) * 600) / 600;
    const bool moving = trace.tick(angle, 0.0);
    if (i >= 30) {EXPECT_TRUE(moving);}
  }
}
}  // namespace
