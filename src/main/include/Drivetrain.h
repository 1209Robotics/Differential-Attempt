// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/motorcontrol/MotorControllerGroup.h>



#include <ctre/Phoenix.h>
#include <AHRS.h>
#include "Constants.h"

using namespace Constants;

/**
 * Represents a differential drive style drivetrain.
 */

class Drivetrain {
 public:
  Drivetrain() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightGroup.SetInverted(true);

    m_leftLeader.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0 , 10);
    m_rightLeader.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);

    m_gyro.Reset();

  }


  void SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds);
  void Drive(units::meters_per_second_t xSpeed,
             units::radians_per_second_t rot);
  void UpdateOdometry();

  double getLeftVelocity();
  double getRightVelocity();
  units::length::meter_t getLeftDistance();
  units::length::meter_t getRightDistance();


 private:
  WPI_TalonFX m_leftLeader{7};
  WPI_TalonFX m_leftFollower{9};
  WPI_TalonFX m_rightLeader{6};
  WPI_TalonFX m_rightFollower{8};

  frc::MotorControllerGroup m_leftGroup{m_leftLeader, m_leftFollower};
  frc::MotorControllerGroup m_rightGroup{m_rightLeader, m_rightFollower};


  frc2::PIDController m_leftPIDController{1.0, 0.0, 0.0};
  frc2::PIDController m_rightPIDController{1.0, 0.0, 0.0};

  AHRS m_gyro{frc::SPI::Port::kMXP};

  frc::DifferentialDriveKinematics m_kinematics{kTrackWidth};
  frc::DifferentialDriveOdometry m_odometry{m_gyro.GetRotation2d()};

  // Gains are for example purposes only - must be determined for your own
  // robot!
    frc::SimpleMotorFeedforward<units::meters> m_feedforward{
        kDriveFeedforwardKs,
        kDriveFeedforwardKv,
        kDriveFeedforwardKa};


                                              
};
