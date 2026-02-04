// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/trajectory/TrapezoidProfile.h>
#include <rev/SparkMax.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>

#include <numbers>

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants {
// Driving Parameters - Note that these are not the maximum capable speeds of
// the robot, rather the allowed maximum speeds
constexpr units::meters_per_second_t kMaxSpeed = 0.5_mps; //4.8
constexpr units::radians_per_second_t kMaxAngularSpeed{1 * std::numbers::pi};

constexpr double kDirectionSlewRate = 1.2;   // radians per second
constexpr double kMagnitudeSlewRate = 1.8;   // percent per second (1 = 100%)
constexpr double kRotationalSlewRate = 2.0;  // percent per second (1 = 100%)

// Chassis configuration

// Distance between centers of right and left wheels on robot
constexpr units::meter_t kTrackWidth = 22.5_in;  
// Distance between centers of front and back wheels on robot
constexpr units::meter_t kWheelBase = 25.5_in;  

// Angular offsets of the modules relative to the chassis in radians
constexpr double kFrontLeftChassisAngularOffset = -std::numbers::pi / 2;
constexpr double kFrontRightChassisAngularOffset = 0;
constexpr double kRearLeftChassisAngularOffset = std::numbers::pi;
constexpr double kRearRightChassisAngularOffset = std::numbers::pi / 2;

//constexpr bool kFieldRelative = true;

// SPARK MAX CAN IDs
constexpr int kRearLeftDrivingCanId = 2;
constexpr int kRearLeftTurningCanId = 3;

constexpr int kFrontLeftDrivingCanId = 4;
constexpr int kFrontLeftTurningCanId = 5;

constexpr int kFrontRightDrivingCanId = 6;
constexpr int kFrontRightTurningCanId = 7;

constexpr int kRearRightDrivingCanId = 8;
constexpr int kRearRightTurningCanId = 9;

constexpr int kPigeonCanId = 10;
}  // namespace DriveConstants

namespace ModuleConstants {
// The MAXSwerve module can be configured with one of three pinion gears: 12T,
// 13T, or 14T. This changes the drive speed of the module (a pinion gear with
// more teeth will result in a robot that drives faster).
constexpr int kDrivingMotorPinionTeeth = 14;

// Calculations required for driving motor conversion factors and feed forward
constexpr double kDrivingMotorFreeSpeedRps = 5676.0 / 60;  // NEO free speed is 5676 RPM
constexpr units::meter_t kWheelDiameter = 0.0762_m;
constexpr units::meter_t kWheelCircumference = kWheelDiameter * std::numbers::pi;
// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
// teeth on the bevel pinion
constexpr double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
constexpr double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumference.value()) / kDrivingMotorReduction;
}  // namespace ModuleConstants

namespace AutoConstants {
constexpr auto kMaxSpeed = 3_mps;
constexpr auto kMaxAcceleration = 3_mps_sq;
constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

extern const frc::TrapezoidProfile<units::radians>::Constraints kThetaControllerConstraints;
}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
constexpr int kCoDriverControllerPort = 1;
constexpr double kDriveDeadband = 0.05;
}  // namespace OIConstants

namespace ShooterConstants {
    constexpr int kShooterLeftCanId = 15;
    constexpr int kShooterRightCanId = 14;
    constexpr int kFeederCanId = 11;
}
//A
namespace TurretConstants {
    constexpr int kTurretCanId = 24;
}

namespace ClimberConstants {
    constexpr int kClimberCanId = 17;
}

namespace IntakeConstants {
    constexpr int kIntakeCanId = 13; 
}

namespace AprilTagConstants {
    // Red Alliance Tags
    namespace Red {
        constexpr int kOutpost = 13;
        constexpr int kLadder1 = 15;
        constexpr int kLadder2 = 16;
        constexpr int kHubAllianceSide1 = 9;   // Facing alliance zone
        constexpr int kHubAllianceSide2 = 10;  // Facing alliance zone
        constexpr int kHubNeutralSide1 = 3;    // Facing neutral zone
        constexpr int kHubNeutralSide2 = 4;    // Facing neutral zone
    }
    // Blue Alliance Tags
    namespace Blue {
        constexpr int kOutpost = 29;
        constexpr int kLadder1 = 31;
        constexpr int kLadder2 = 32;
        constexpr int kHubAllianceSide1 = 25;  // Facing alliance zone
        constexpr int kHubAllianceSide2 = 26;  // Facing alliance zone
        constexpr int kHubNeutralSide1 = 19;   // Facing neutral zone
        constexpr int kHubNeutralSide2 = 20;   // Facing neutral zone
    }
}

namespace AutoTimingConstants {
    constexpr double kHumanPlayerWaitSeconds = 3.0;
    constexpr double kObstacleAngleDegrees = 45.0;  // Chassis angle for obstacle crossing (35-55 range)
}