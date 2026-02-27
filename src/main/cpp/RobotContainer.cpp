// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/DriverStation.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/Commands.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <cmath>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include <utility>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"


using namespace DriveConstants;
using namespace pathplanner;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

    NamedCommands::registerCommand("Shoot", std::move(m_shooter.ShootAuto()));
    NamedCommands::registerCommand("StopShooting", std::move(m_shooter.StopAuto()));
    NamedCommands::registerCommand("ClimberUp", std::move(m_climber.UpAuto()));
    NamedCommands::registerCommand("ClimberDown", std::move(m_climber.DownAuto()));

  // Auto chooser
    m_chooser.SetDefaultOption("shootClimb", "shootClimb");
    frc::SmartDashboard::PutData("Auto Selector", &m_chooser);

  // Configure the button bindings
    ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
    auto team = frc::DriverStation::GetAlliance();
    if(team.has_value() && team.value() == frc::DriverStation::Alliance::kRed){
        m_camera.SetPriorityTag(AprilTags::Hub::kRedCenter);
    }
    else{
        m_camera.SetPriorityTag(AprilTags::Hub::kBlueCenter);
    }
    m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_drive.Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftY(), OIConstants::kDriveDeadband)},
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftX(), OIConstants::kDriveDeadband)},
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController.GetRightX(), OIConstants::kDriveDeadband)},
            fieldRelative);
      },
      {&m_drive}));

    m_camera.SetDefaultCommand(frc2::RunCommand([this]{m_camera.PutStuffOnSmartDashboard();},{&m_camera}));

    m_LEDs.SetDefaultCommand(frc2::RunCommand([this]{
        if((m_camera.GetDetection() == true) ){
            double distance = m_camera.GetDistance();
                if((distance >= 5) && (distance <= 15)){
                    if(((m_camera.GetTagId()==AprilTags::Hub::kBlueCenter) || m_camera.GetTagId()==AprilTags::Hub::kRedCenter) && m_camera.GetDetection()){
                        m_coDriverController.SetRumble(frc::GenericHID::kRightRumble, 0.015);
                        m_LEDs.TurnOnLEDs(0.0f, 0.5f, 0.0f); // If the camera sees an AprilTag, sets lights to green
                    }
                }
        } else{
            m_coDriverController.SetRumble(frc::GenericHID::kBothRumble, 0.0);
            auto team = frc::DriverStation::GetAlliance();
            if(team.has_value() && team.value() == frc::DriverStation::Alliance::kRed){ m_LEDs.TurnOnLEDs(1.0f, 0.0f, 0.0f); }
            else{ m_LEDs.TurnOnLEDs(0.0f, 0.0f, 1.0f); }
            //m_LEDs.GO(0,0,0);
        }
    },{&m_camera, &m_LEDs}));

    m_intake.SetDefaultCommand(frc2::RunCommand([this]{
        int dPOV = m_driverController.GetPOV();
        if(dPOV == -1){ //no pov pressed
            if(m_coDriverController.GetBButton()){
               m_intake.Reverse();
            }
            else{
                m_intake.Stop();
            }
        }
        else if(dPOV == 180){ //down
            m_intake.LowerLifter();
        }
        else if(dPOV == 0){ // up
            m_intake.RaiseLifter();
        }
        else{
            m_intake.Stop();
        }
    
    },{&m_intake}));

    m_turret.SetDefaultCommand(frc2::RunCommand([this]{
        if(m_coDriverController.GetLeftStickButton()){
            if(m_camera.GetDetection()){
                 m_turret.PointAtAprilTag(-m_camera.GetYaw());
            }
            else{
                m_turret.SetSpeed(0);
            }
        }
        else{
                m_turret.SetSpeed(m_coDriverController.GetLeftX()*0.2);
            }

    },{&m_turret}));

    m_climber.SetDefaultCommand(frc2::RunCommand([this]{
        if(m_driverController.GetRightBumperButton()){
            m_climber.Run();
        }
        else if(m_driverController.GetLeftBumperButton()){
            m_climber.Reverse();
        }
        else if(m_driverController.GetPOV() == 90){
            m_climber.ReverseBypass();
        }
        else{
            m_climber.Stop();
        }
        //frc::SmartDashboard::PutBoolean("climber limit switch", m_ClimberLimitSwitch.Get());
    },{&m_climber}));
}
//wade is a [rogramer]
//wade is not a [BIG SHOT]
//wade is a {small shot}
//now's your chance now's your chance to be a {small shot}
void RobotContainer::ConfigureButtonBindings() {
    /*frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kRightBumper).WhileTrue
    (new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));*/

    frc2::JoystickButton(&m_coDriverController, frc::XboxController::Button::kRightBumper).OnTrue(
        new frc2::InstantCommand([this] {
            if(m_camera.GetDetection()){
                double distance = m_camera.GetDistance();
                if((distance > 5) && (distance < 15)){
                    m_shooter.RunCollector();
                    m_shooter.Shoot((110*m_camera.GetDistance()) + 2300);
                }
                
            }
            else{
                m_shooter.RunCollector();
                m_shooter.Shoot();
            }

            
        
        },{&m_shooter})).OnFalse(
            new frc2::InstantCommand([this] {m_shooter.Stop();}, {&m_shooter})
    );

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kStart).OnTrue
    (new frc2::InstantCommand([this] {fieldRelative = !fieldRelative;}));

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kBack).OnTrue
    (new frc2::InstantCommand([this] {m_drive.ZeroHeading();},{&m_drive}));

    frc2::JoystickButton(&m_coDriverController, frc::XboxController::Button::kY).OnTrue(new frc2::InstantCommand([this]{
        m_shooter.ReverseCollector();
    },{&m_shooter})).OnFalse(new frc2::InstantCommand([this]{m_shooter.StopCollector();},{&m_shooter}));
    /*frc2::JoystickButton(&m_coDriverController, frc::XboxController::Button::kY).OnTrue(new frc2::InstantCommand([this]{
        m_shooter.RunCollector();
    },{&m_shooter})).OnFalse(new frc2::InstantCommand([this]{m_shooter.StopCollector();},{&m_shooter}));*/
}

void RobotContainer::StopAll() {
    m_shooter.Stop();
    m_shooter.StopCollector();
    m_intake.Stop();
    m_climber.Stop();
    m_turret.SetSpeed(0);
}

// To add a new auto routine:
//   1. Add a new method like GetShootClimbAuto() that returns frc2::CommandPtr
//   2. Declare it in RobotContainer.h
//   3. In the constructor, add: m_chooser.AddOption("myAuto", "myAuto");
//   4. Add an "if" branch below to call your new method
frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
    std::string selected = m_chooser.GetSelected();

    if (selected == "shootClimb" || selected.empty()) {
        return GetShootClimbAuto();
    }

    // Default fallback
    return GetShootClimbAuto();
}

frc2::CommandPtr RobotContainer::GetShootClimbAuto() {
    using namespace AutonomousRoutine;

    return frc2::cmd::Sequence(

        // Reset gyro heading at the start of auto
        frc2::InstantCommand([this] { m_drive.ZeroHeading(); }, {&m_drive}).ToPtr(),

        // Phase 1: Drive forward 12 inches
        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] {
                    m_drive.ResetOdometry(frc::Pose2d{});
                    m_autoTargetHeading = m_drive.GetYawDegrees();
                },
                [this] {
                    double rotCorrection = 0.0;
                    if (kHeadingCorrectionEnabled) {
                        double headingError = m_autoTargetHeading - m_drive.GetYawDegrees();
                        rotCorrection = headingError * kHeadingCorrectionPGain;
                    }
                    m_drive.driveRobotRelative(
                        frc::ChassisSpeeds{units::meters_per_second_t{kDriveSpeed}, 0_mps,
                            units::radians_per_second_t{rotCorrection}});
                },
                [this](bool) {
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                },
                [this] {
                    double dist = m_drive.GetPose().Translation().Norm().value();
                    return dist >= kDriveDistance1_ft * 0.3048;
                },
                {&m_drive}
            ).ToPtr(),
            frc2::WaitCommand(units::second_t{kDriveTimeout_s}).ToPtr()
        ),

        // Phase 2: Rotate 25 degrees left
        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] { m_autoStartHeading = m_drive.GetYawDegrees(); },
                [this] {
                    double error = (m_autoStartHeading + kRotateAngleDeg) - m_drive.GetYawDegrees();
                    double rotSpeed = std::clamp(error * kRotatePGain, -1.0, 1.0);
                    m_drive.driveRobotRelative(
                        frc::ChassisSpeeds{0_mps, 0_mps, units::radians_per_second_t{rotSpeed}});
                },
                [this](bool) {
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                },
                [this] {
                    double error = std::abs((m_autoStartHeading + kRotateAngleDeg) - m_drive.GetYawDegrees());
                    return error <= kRotateToleranceDeg;
                },
                {&m_drive}
            ).ToPtr(),
            frc2::WaitCommand(units::second_t{kRotateTimeout_s}).ToPtr()
        ),

        // Phase 3: Shoot for 7 seconds
        frc2::InstantCommand(
            [this] {
                m_shooter.Shoot(kShootRPM);
                m_shooter.RunCollector();
            },
            {&m_shooter}
        ).ToPtr(),

        frc2::WaitCommand(units::second_t{kShootDuration_s}).ToPtr(),

        // Phase 4: Stop shooting, rotate back to original heading
        frc2::InstantCommand(
            [this] { m_shooter.Stop(); },
            {&m_shooter}
        ).ToPtr(),

        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] {},
                [this] {
                    double error = m_autoStartHeading - m_drive.GetYawDegrees();
                    double rotSpeed = std::clamp(error * kRotatePGain, -1.0, 1.0);
                    m_drive.driveRobotRelative(
                        frc::ChassisSpeeds{0_mps, 0_mps, units::radians_per_second_t{rotSpeed}});
                },
                [this](bool) {
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                },
                [this] {
                    return std::abs(m_autoStartHeading - m_drive.GetYawDegrees()) <= kRotateToleranceDeg;
                },
                {&m_drive}
            ).ToPtr(),
            frc2::WaitCommand(units::second_t{kRotateTimeout_s}).ToPtr()
        ),

        // Phase 5: Drive forward 3 feet
        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] {
                    m_autoPhase5StartX = m_drive.GetPose().X().value();
                    m_autoPhase5StartY = m_drive.GetPose().Y().value();
                    m_autoTargetHeading = m_drive.GetYawDegrees();
                },
                [this] {
                    double rotCorrection = 0.0;
                    if (kHeadingCorrectionEnabled) {
                        double headingError = m_autoTargetHeading - m_drive.GetYawDegrees();
                        rotCorrection = headingError * kHeadingCorrectionPGain;
                    }
                    m_drive.driveRobotRelative(
                        frc::ChassisSpeeds{units::meters_per_second_t{kDriveSpeed}, 0_mps,
                            units::radians_per_second_t{rotCorrection}});
                },
                [this](bool) {
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                },
                [this] {
                    double dx = m_drive.GetPose().X().value() - m_autoPhase5StartX;
                    double dy = m_drive.GetPose().Y().value() - m_autoPhase5StartY;
                    return std::sqrt(dx*dx + dy*dy) >= kDriveDistance2_ft * 0.3048;
                },
                {&m_drive}
            ).ToPtr(),
            frc2::WaitCommand(units::second_t{kDriveTimeout_s}).ToPtr()
        ),

        // Phase 6: Set priority AprilTag based on alliance, raise climber
        frc2::InstantCommand(
            [this] {
                auto alliance = frc::DriverStation::GetAlliance();
                if (alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kRed) {
                    m_camera.SetPriorityTag(AprilTags::Tower::kRedOffset);
                } else {
                    m_camera.SetPriorityTag(AprilTags::Tower::kBlueOffset);
                }
                m_climber.Run(); // Raise climber to position 60
            },
            {&m_camera, &m_climber}
        ).ToPtr(),

        // Phase 7: Drive forward until camera distance to tag reaches target
        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] {},
                [this] {
                    if (m_camera.GetDetection()) {
                        double yawCorrection = m_camera.GetYaw() * kAprilTagYawPGain;
                        m_drive.driveRobotRelative(
                            frc::ChassisSpeeds{
                                units::meters_per_second_t{kAprilTagDriveSpeed},
                                0_mps,
                                units::radians_per_second_t{yawCorrection}});
                    } else {
                        m_drive.driveRobotRelative(
                            frc::ChassisSpeeds{
                                units::meters_per_second_t{kAprilTagDriveSpeed * 0.5},
                                0_mps, 0_rad_per_s});
                    }
                },
                [this](bool) {
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                },
                [this] {
                    return m_camera.GetDetection() && m_camera.GetDistance() <= kAprilTagTargetDistance_ft;
                },
                {&m_drive, &m_camera}
            ).ToPtr(),
            frc2::WaitCommand(units::second_t{kDriveToTagTimeout_s}).ToPtr()
        ),

        // Phase 8: Strafe left until velocity stall (contact with ladder)
        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] {
                    m_autoStallCount = 0;
                    m_autoTargetHeading = m_drive.GetYawDegrees();
                },
                [this] {
                    double rotCorrection = 0.0;
                    if (kHeadingCorrectionEnabled) {
                        double headingError = m_autoTargetHeading - m_drive.GetYawDegrees();
                        rotCorrection = headingError * kHeadingCorrectionPGain;
                    }
                    m_drive.driveRobotRelative(
                        frc::ChassisSpeeds{0_mps, units::meters_per_second_t{kStrafeSpeed},
                            units::radians_per_second_t{rotCorrection}});
                    if (m_drive.GetAverageDriveVelocity() < kStallVelocityThreshold) {
                        m_autoStallCount++;
                    } else {
                        m_autoStallCount = 0;
                    }
                },
                [this](bool) {
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                },
                [this] {
                    return m_autoStallCount >= kStallConsecutiveCycles;
                },
                {&m_drive}
            ).ToPtr(),
            frc2::WaitCommand(units::second_t{kStrafeTimeout_s}).ToPtr()
        ),

        // Phase 9: Back up until limit switch triggered
        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] { m_autoTargetHeading = m_drive.GetYawDegrees(); },
                [this] {
                    double rotCorrection = 0.0;
                    if (kHeadingCorrectionEnabled) {
                        double headingError = m_autoTargetHeading - m_drive.GetYawDegrees();
                        rotCorrection = headingError * kHeadingCorrectionPGain;
                    }
                    m_drive.driveRobotRelative(
                        frc::ChassisSpeeds{units::meters_per_second_t{-kBackupSpeed}, 0_mps,
                            units::radians_per_second_t{rotCorrection}});
                },
                [this](bool) {
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                },
                [this] {
                    return !m_ClimberLimitSwitch.Get();
                },
                {&m_drive}
            ).ToPtr(),
            frc2::WaitCommand(units::second_t{kBackupTimeout_s}).ToPtr()
        ),

        // Phase 10: Lower climber to climb
        frc2::InstantCommand(
            [this] { m_climber.Reverse(); }, // Lower to position 0
            {&m_climber}
        ).ToPtr()

        

    );
}
