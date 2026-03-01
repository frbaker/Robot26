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
#include <list>

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
constexpr auto kMaxSpeed = 1_mps;
constexpr auto kMaxAcceleration = 1_mps_sq;
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
    constexpr int kShooterLeftCanId = 13;
    constexpr int kShooterRightCanId = 14;
    constexpr int kFeederCanId = 11;
    constexpr int kCollectorCanId = 19;

    constexpr int kShooterRPM = 3000;
    constexpr int kShooterVeloTolerance = 100; // Subject to change 
}
//A
namespace TurretConstants {
    constexpr int kTurretCanId = 17;
    constexpr double kTurretMinimum = -8;
    constexpr double kTurretMaximum = 8.7;
}

namespace ClimberConstants {
    constexpr int kClimberCanId = 15; //Idk lol
}
namespace IntakeConstants {
    constexpr int kIntakeCanId = 16;
    constexpr int kLifterCanId = 18;
}

namespace LEDConstants {
    constexpr int kRedPin = 3;
    constexpr int kGreenPin = 4;
    constexpr int kBluePin = 5;
}
namespace AutonomousRoutine {
    // Phase 1: Drive forward 12 inches
    constexpr double kDriveDistance1_ft = 10.33;

    // Phase 2: Rotate left
    constexpr double kRotateAngleDeg = 25.0;

    // Phase 3: Shoot
    constexpr double kShootDuration_s = 7.0;
    constexpr double kShootRPM = 3200.0;

    // Phase 5: Drive forward 3 feet
    constexpr double kDriveDistance2_ft = 3.0;

    // Phase 7: Drive to AprilTag
    constexpr double kAprilTagTargetDistance_ft = 2.0; // TBD - tune on robot
    constexpr double kAprilTagDriveSpeed = 0.3;        // m/s
    constexpr double kAprilTagYawPGain = 0.01;

    // Camera-guided alignment before stall detection
    constexpr double kAlignStrafePGain = 0.01;
    constexpr double kAlignDistancePGain = 0.1;
    constexpr double kAlignTargetDistance_ft = 2.0;
    constexpr double kAlignYawTolerance = 2.0;          // degrees
    constexpr double kAlignDistanceTolerance_ft = 0.3;   // feet
    constexpr double kAlignTimeout_s = 4.0;

    // Phase 8: Strafe left to tower
    constexpr double kStrafeSpeed = 0.1;               // m/s
    constexpr double kStrafeDistance_ft = 1.2;         // feet
    constexpr double kStrafeTimeout_s = 2.5;           // seconds

    // Phase 9: Back up to limit switch
    constexpr double kBackupSpeed = 0.02;               // m/s
    constexpr int kLimitSwitchChannel = 1;

    // Heading correction during straight-line driving
    constexpr bool kHeadingCorrectionEnabled = true;
    constexpr double kHeadingCorrectionPGain = 0.02; //increase if robot drifts off course, decrease if it oscillates

    // Turret auto aiming
    constexpr double kTurretAimYawTolerance = 2.0;   // degrees
    constexpr double kTurretReturnPGain = 0.1;
    constexpr double kTurretReturnTolerance = 0.5;    // encoder units

    // General driving
    constexpr double kDriveSpeed = 0.1;                // m/s
    constexpr double kRotatePGain = 0.005;
    constexpr double kRotateToleranceDeg = 2.0;
    constexpr double kDistanceToleranceMeters = 0.03;

    // Safety timeouts
    constexpr double kDriveTimeout_s = 6.0;
    constexpr double kRotateTimeout_s = 2.5;
    constexpr double kDriveToTagTimeout_s = 8.0;
    constexpr double kBackupTimeout_s = 5.0;

    namespace RightBumpShootClimb{
        constexpr bool kHeadingCorrectionEnabled = true;
        constexpr double kHeadingCorrectionPGain = 0.02;
        constexpr double kBackupSpeed = 0.02;
        constexpr double kBackupTimeout_s = 12.0;
        constexpr double kDrive1Speed = 0.1;
        constexpr double kDriveTimeout_s = 6;
        constexpr double kDriveDistance1_ft = 9;
        constexpr double kStrafeDistance_ft = 2.2;
        constexpr double kStrafeSpeed = 0.05;
        constexpr double kStrafeTimeout_s = 2.5;
    }

    namespace OverBump{
        constexpr bool kHeadingCorrectionEnabled = true;
        constexpr double kHeadingCorrectionPGain = 0.02;
        constexpr double kDriveSpeed = 0.2;
        constexpr double kDriveDistance1_ft = 10;
        constexpr double kDriveTimeout_s = 5;

        constexpr double kDriveTimeout2_s = 5;
        constexpr double kDriveDistance2_ft = 6.5;
        constexpr double kDriveSpeed2 = 0.2;

        constexpr double kDriveTimeout3_s = 5;
        constexpr double kDriveDistance3_ft = 17;
        constexpr double kDriveSpeed3 = 0.25;

        constexpr double kShootRPM = 2900;
    }
}

namespace AprilTags{
    namespace Hub{
        constexpr int kRedCenter = 10;
        constexpr int kRedOffset = 9;
        constexpr int kBlueCenter = 26;
        constexpr int kBlueOffset = 25;
    }
    namespace Tower{
        constexpr int kRedCenter = 15;
        constexpr int kRedOffset = 16;
        constexpr int kBlueCenter = 31;
        constexpr int kBlueOffset = 32;
    }
    namespace Outpost{
        constexpr int kRedCenter = 13;
        constexpr int kRedOffset = 14;
        constexpr int kBlueCenter = 29;
        constexpr int kBlueOffset = 30;
    }
}