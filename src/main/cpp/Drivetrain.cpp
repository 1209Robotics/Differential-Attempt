// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"

void Drivetrain::SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds) {
    const auto leftFeedforward = m_feedforward.Calculate(speeds.left);
    const auto rightFeedforward = m_feedforward.Calculate(speeds.right);
    const double leftOutput = m_leftPIDController.Calculate(
        getLeftVelocity(), speeds.left.value());
    const double rightOutput = m_rightPIDController.Calculate(
      getRightVelocity(), speeds.right.value());

  m_leftGroup.SetVoltage(units::volt_t{leftOutput} + leftFeedforward);
  m_rightGroup.SetVoltage(units::volt_t{rightOutput} + rightFeedforward);
  
}

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::radians_per_second_t rot) {
  SetSpeeds(m_kinematics.ToWheelSpeeds({xSpeed, 0_mps, rot}));
}


double Drivetrain::getLeftVelocity()
{
    double sensorCounts = m_leftLeader.GetSelectedSensorVelocity() * 10;
    //converts the sensor counts (which are out of 2048) into a rotation around the motor shaft
    double motorRotationsPerSecond {sensorCounts / kEncoderResolution * 2 * wpi::numbers::pi};
    //converts motor shaft outputs to wheel shaft outputs
    double wheelRotations {motorRotationsPerSecond / kGearRatio};
    //converts wheel shaft rotations into a distance by multiplying by the radius of the wheel
    double positionMeters{wheelRotations * kWheelRadius};
    return positionMeters;

}

double Drivetrain::getRightVelocity()
{
    double sensorCounts = m_rightLeader.GetSelectedSensorVelocity() * 10;
    //converts the sensor counts (which are out of 2048) into a rotation around the motor shaft
    double motorRotationsPerSecond {sensorCounts / kEncoderResolution * 2 * wpi::numbers::pi};
    //converts motor shaft outputs to wheel shaft outputs
    double wheelRotations {motorRotationsPerSecond / kGearRatio};
    //converts wheel shaft rotations into a distance by multiplying by the radius of the wheel
    double positionMeters{wheelRotations * kWheelRadius};
    return positionMeters;
    

}
units::length::meter_t Drivetrain::getLeftDistance()
{
    double sensorCounts = m_leftLeader.GetSelectedSensorPosition();
    double motorRotations = sensorCounts / kEncoderResolution;
    double wheelRotations = motorRotations / kGearRatio;
    units::meter_t positionMeters{wheelRotations * 2 * wpi::numbers::pi * kWheelRadius};
    return positionMeters;

}
units::length::meter_t Drivetrain::getRightDistance()
{
    double sensorCounts = m_leftLeader.GetSelectedSensorPosition();
    double motorRotations = sensorCounts / kEncoderResolution;
    double wheelRotations = motorRotations / kGearRatio;
    units::meter_t positionMeters{wheelRotations * 2 * wpi::numbers::pi * kWheelRadius};
    return positionMeters;

}

void Drivetrain::UpdateOdometry() {
  m_odometry.Update(m_gyro.GetRotation2d(),
                    getLeftDistance(),
                    getRightDistance());
}
