#pragma once

#include <wpi/numbers>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/area.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>



namespace Constants 
{
  static constexpr units::meter_t kTrackWidth = 23_in;
  static constexpr units::meter_t kWheelRadius = 0.0762_m;  // meters
  static constexpr double_t kEncoderResolution = 2048;
  static constexpr double_t kGearRatio = 10.71;

  static constexpr units::meters_per_second_t kMaxSpeed =
      3.0_mps;  // 3 meters per second
  static constexpr units::radians_per_second_t kMaxAngularSpeed{
      wpi::numbers::pi};  // 1/2 rotation per second


    constexpr units::volt_t kDriveFeedforwardKs{0.56708};
    constexpr units::unit_t<units::compound_unit<units::volts, units::inverse<units::meters_per_second>>> kDriveFeedforwardKv {2.4072};
    constexpr units::unit_t<units::compound_unit<units::volts, units::inverse<units::meters_per_second_squared>>> kDriveFeedforwardKa{0.56708};

}