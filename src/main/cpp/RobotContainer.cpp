// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <cmath>

#include <utility>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
    ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
    m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        double xSpeed = -frc::ApplyDeadband(
            m_driverController.GetLeftY(), OIConstants::kDriveDeadband);
        double ySpeed = -frc::ApplyDeadband(
            m_driverController.GetLeftX(), OIConstants::kDriveDeadband);
        double rotSpeed = -frc::ApplyDeadband(
            m_driverController.GetRightX(), OIConstants::kDriveDeadband);

        m_drive.Drive(
            units::meters_per_second_t{xSpeed},
            units::meters_per_second_t{ySpeed},
            units::radians_per_second_t{rotSpeed},
            DriveConstants::kFieldRelative);

        // Update LEDs with robot state
        m_leds.SetHeading(m_drive.GetHeading());
        double speed = std::sqrt(xSpeed * xSpeed + ySpeed * ySpeed);
        m_leds.SetSpeed(speed);
        m_leds.SetDriveVector(xSpeed, ySpeed);
      },
      {&m_drive}));
    m_camera.SetDefaultCommand(frc2::RunCommand([this]{m_camera.PutStuffOnSmartDashboard();},{&m_camera}));
}

void RobotContainer::ConfigureButtonBindings() {
    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kRightBumper).WhileTrue
    (new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));

    // ==========================================================================
    // ISSUE: SHOOTER BUTTON BINDING HAS TWO PROBLEMS
    // ==========================================================================
    //
    // PROBLEM 1: Shoot() is called every scheduler cycle (~50Hz)
    // -------------------------------------------------------------
    // WhileTrue(RunCommand) means Shoot() runs repeatedly while held.
    // This works, but is inefficient because SetReference() is called
    // every 20ms even though the target velocity never changes.
    //
    // PROBLEM 2: Stop() RunCommand runs forever after button release
    // -------------------------------------------------------------
    // OnFalse(RunCommand) creates a command that calls Stop() repeatedly
    // with no end condition. RunCommand only ends when interrupted, so
    // Stop() will be called every cycle until another command takes over
    // the shooter subsystem. This wastes CPU cycles.
    //
    // ==========================================================================
    // RECOMMENDED FIX: Use StartEnd or InstantCommand
    // ==========================================================================
    //
    // Option A: Using StartEnd (cleanest approach)
    // ---------------------------------------------
    // StartEnd runs one lambda when the command starts, another when it ends.
    // The command ends automatically when the button is released.
    //
    //   frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kA)
    //       .WhileTrue(frc2::StartEndCommand(
    //           [this] { m_shooter.Shoot(); },   // Called once on button press
    //           [this] { m_shooter.Stop(); },    // Called once on button release
    //           {&m_shooter}
    //       ).ToPtr());
    //
    // Option B: Using OnTrue/OnFalse with InstantCommand
    // ---------------------------------------------------
    // InstantCommand runs once and immediately finishes.
    // Good when you want fire-and-forget behavior.
    //
    //   frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kA)
    //       .OnTrue(frc2::InstantCommand([this] { m_shooter.Shoot(); }, {&m_shooter}).ToPtr())
    //       .OnFalse(frc2::InstantCommand([this] { m_shooter.Stop(); }, {&m_shooter}).ToPtr());
    //
    // Option C: Create a dedicated ShootCommand class
    // ------------------------------------------------
    // For more complex behavior (e.g., wait until at speed before feeding),
    // create a proper command class with Initialize(), Execute(), End(),
    // and IsFinished() methods. This gives full control over the lifecycle.
    //
    // ==========================================================================

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kA).WhileTrue(
        new frc2::RunCommand([this] {m_shooter.Shoot();},{&m_shooter})).OnFalse(
            new frc2::RunCommand([this] {m_shooter.Stop();}, {&m_shooter})
    );

}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{0_m, 0_m, 0_deg},
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d{1_m, 1_m}, frc::Translation2d{2_m, -1_m}},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d{3_m, 0_m, 0_deg},
      // Pass the config
      config);

  frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                        units::radian_t{std::numbers::pi});

  frc2::SwerveControllerCommand<4> swerveControllerCommand(
      exampleTrajectory, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc::PIDController{AutoConstants::kPXController, 0, 0},
      frc::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(exampleTrajectory.InitialPose());

  // no auto
  return new frc2::SequentialCommandGroup(
      std::move(swerveControllerCommand),
      frc2::InstantCommand(
          [this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); }, {}));
}
