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
#include <iostream>

#include <utility>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"


using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Auto chooser
    m_chooser.SetDefaultOption("shootClimb", "shootClimb");
    m_chooser.AddOption("overBump", "overBump");
    frc::SmartDashboard::PutData("Auto Selector", &m_chooser);

  // Configure the button bindings
    ConfigureButtonBindings();
    ConfigureAlliance();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
    m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        frc::SmartDashboard::PutNumber("Drive Heading", m_drive.GetHeading().value());
        auto xSpeed = -units::meters_per_second_t{frc::ApplyDeadband(
            m_driverController.GetLeftY(), OIConstants::kDriveDeadband)};
        auto ySpeed = -units::meters_per_second_t{frc::ApplyDeadband(
            m_driverController.GetLeftX(), OIConstants::kDriveDeadband)};

        units::radians_per_second_t rot{0.0};

        if (m_coDriverController.GetLeftBumperButton()) {
            // Auto-aim mode: camera controls rotation, driver keeps translation
            if (m_isRedAlliance) {
                m_camera.SetPriorityTag(AprilTags::Hub::kRedCenter);
            } else {
                m_camera.SetPriorityTag(AprilTags::Hub::kBlueCenter);
            }
            if (m_camera.GetDetection()) {
                rot = m_drive.CameraDrive(-m_camera.GetYaw());
            }
            // If no detection, rot stays 0 (hold still rotationally)
        } else if (m_driverController.GetAButton() || m_driverController.GetBButton()) {
            if (!m_teleSpinActive) {
                double direction = m_driverController.GetAButton() ? 180.0 : -180.0;
                m_teleSpinTarget = m_drive.GetYawDegrees() + direction;
                m_teleSpinActive = true;
            }
            double headingError = m_teleSpinTarget - m_drive.GetYawDegrees();
            double rotOverride = std::clamp(headingError * OIConstants::kSpinPGain, -OIConstants::kSpinClamp, OIConstants::kSpinClamp);
            rot = units::radians_per_second_t{rotOverride};
        } else {
            m_teleSpinActive = false;
            rot = -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController.GetRightX(), OIConstants::kDriveDeadband)};
        }

        m_drive.Drive(xSpeed, ySpeed, rot, fieldRelative);
      },
      {&m_drive}));

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

    m_climber.SetDefaultCommand(frc2::RunCommand([this]{
        if(m_driverController.GetRightBumperButton()){
            m_climber.Run();
        }
        else if(m_driverController.GetLeftBumperButton()){
            m_climber.Reverse();
        }
        else if(m_driverController.GetPOV() == 90){ //right
            m_climber.ReverseBypass();
        }
        else if(m_driverController.GetPOV() == 270){ //left
            m_climber.RunBypass();
        }
        else{
            m_climber.Stop();
        }
        frc::SmartDashboard::PutBoolean("climber limit switch", m_ClimberLimitSwitch.Get());
    },{&m_climber}));
}
//wade is a [rogramer]
//wade is not a [BIG SHOT]
//wade is a {small shot}
//now's your chance now's your chance to be a {small shot}
void RobotContainer::ConfigureButtonBindings() {
    /*frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kRightBumper).WhileTrue
    (new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));*/

    /*4ft 2750rpm
    8ft 3275rpm
    12ft 3800rpm

    131 per ft

    x = 12-ft
    y = x*131
    z = 3800-y

    power = z

    over bump auto 2900rpm*/


    frc2::JoystickButton(&m_coDriverController, frc::XboxController::Button::kRightBumper).OnTrue(
        frc2::cmd::Sequence(
            frc2::InstantCommand([this] {
                if(m_camera.GetDetection()){
                    double distance = m_camera.GetDistance();
                    if((distance > 0) && (distance < 2000)){ //placeholder values
                        //m_shooter.Shoot(3800-((12-distance)*131));
                        m_shooter.Shoot();
                    }
                    else{
                        m_shooter.Shoot();
                    }
                } else {
                    m_shooter.Shoot();
                }
            }, {&m_shooter}).ToPtr(),
            frc2::WaitCommand(units::second_t{1}).ToPtr(),
            frc2::InstantCommand([this] {
                m_shooter.RunCollector();
            }, {&m_shooter}).ToPtr()
        )
    ).OnFalse(
            new frc2::InstantCommand([this] {m_shooter.Stop();}, {&m_shooter})
    );

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kStart).OnTrue
    (new frc2::InstantCommand([this] {fieldRelative = !fieldRelative;}));

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kBack).OnTrue
    (new frc2::InstantCommand([this] {m_drive.ZeroHeading();},{&m_drive}));

    frc2::JoystickButton(&m_coDriverController, frc::XboxController::Button::kY).OnTrue(new frc2::InstantCommand([this]{
        m_shooter.ReverseCollector();
        m_shooter.ReverseFeeder();
    },{&m_shooter})).OnFalse(new frc2::InstantCommand([this]{m_shooter.StopCollector(); m_shooter.StopFeeder();},{&m_shooter}));

    frc2::JoystickButton(&m_coDriverController, frc::XboxController::Button::kA).OnTrue(new frc2::InstantCommand([this]{
        m_intake.Run(); //Reverse intake
    },{&m_intake}))
    .OnFalse(new frc2::InstantCommand([this]{
        m_intake.Stop();
    },{&m_intake}));

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kX).WhileTrue(
        new frc2::RunCommand([this]{m_drive.SetX();},{&m_drive}
    ));
    /*frc2::JoystickButton(&m_coDriverController, frc::XboxController::Button::kY).OnTrue(new frc2::InstantCommand([this]{
        m_shooter.RunCollector();
    },{&m_shooter})).OnFalse(new frc2::InstantCommand([this]{m_shooter.StopCollector();},{&m_shooter}));*/
}

void RobotContainer::ConfigureAlliance() {
    auto team = frc::DriverStation::GetAlliance();
    m_isRedAlliance = team.has_value() && team.value() == frc::DriverStation::Alliance::kRed;
    if (m_isRedAlliance) {
        m_camera.SetPriorityTag(AprilTags::Hub::kRedCenter);
    } else {
        m_camera.SetPriorityTag(AprilTags::Hub::kBlueCenter);
    }
}

void RobotContainer::StopAll() {
    m_shooter.Stop();
    m_shooter.StopCollector();
    m_intake.Stop();
    m_climber.Stop();
    m_coDriverController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);
}

// To add a new auto routine:
//   1. Add a new method like GetShootClimbAuto() that returns frc2::CommandPtr
//   2. Declare it in RobotContainer.h
//   3. In the constructor, add: m_chooser.AddOption("myAuto", "myAuto");
//   4. Add an "if" branch below to call your new method
frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
    std::string selected = m_chooser.GetSelected();
    m_drive.ResetOdometry(frc::Pose2d{});

    if (selected == "shootClimb") {
        return GetShootClimbAuto();
    }
    if (selected == "overBump"){
        return GetOverBumpAuto();
    }

    // Default fallback
    return GetShootClimbAuto();
}

frc2::CommandPtr RobotContainer::GetShootClimbAuto() {
    using namespace AutonomousRoutine;
    return frc2::cmd::Sequence(
        // Reset gyro heading at the start of auto
        frc2::InstantCommand([this] { m_drive.ZeroHeading(); m_drive.ResetOdometry(frc::Pose2d{}); }, {&m_drive}).ToPtr(),

        frc2::WaitCommand(units::time::second_t{0.5}).ToPtr(), //give pigeon time to stabilize after reset

        
        // Phase 1: Drive forward while raising climber
        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] {
                    //onInit
                    ConfigureAlliance();
                    m_autoTargetHeading = m_drive.GetYawDegrees();
                },
                [this] {
                    //onExec
                    double rotCorrection = 0.0;
                    if (kHeadingCorrectionEnabled) {
                        double headingError = m_autoTargetHeading - m_drive.GetYawDegrees();
                        rotCorrection = headingError * kHeadingCorrectionPGain;
                    }
                    m_drive.driveRobotRelative(
                        frc::ChassisSpeeds{units::meters_per_second_t{kDriveSpeed}, 0_mps, units::radians_per_second_t{rotCorrection}}
                    );
                    //m_climber.Run();
                },
                [this](bool) {
                    //onEnd
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                    //m_climber.Stop();
                },
                [this] {
                    //isFinished?
                    double dist = m_drive.GetPose().Translation().Norm().value();
                    return dist >= kDriveDistance1_ft * 0.3048;
                },
                {&m_drive, &m_climber}
            ).ToPtr(),
            frc2::WaitCommand(units::second_t{kDriveTimeout_s}).ToPtr()
        ),

        // Phase 2: Aim robot at AprilTag using swerve rotation
        /*frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] {
                    if(m_isRedAlliance){
                        m_camera.SetPriorityTag(AprilTags::Hub::kRedCenter);
                    }
                    else{
                        m_camera.SetPriorityTag(AprilTags::Hub::kBlueCenter);
                    }
                },
                [this] {
                    if (m_camera.GetDetection()) {
                        units::radians_per_second_t aimRot = m_drive.CameraDrive(-m_camera.GetYaw());
                        m_drive.Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, aimRot, false);
                        //m_drive.CameraDrive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, m_camera.GetYaw(), false);
                    } else {
                        m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                    }
                },
                [this](bool) {
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                },
                [this] {
                    return m_camera.GetDetection() &&
                           std::abs(m_camera.GetYaw()) < kDriveAimYawTolerance;
                },
                {&m_drive, &m_camera}
            ).ToPtr(),
            frc2::WaitCommand(units::second_t{kRotateTimeout_s}).ToPtr()
        ),*/
        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] {
                    
                },
                [this] {
                    m_drive.RotateToHeading(kHubRotationTarget);
                },
                [this] (bool) {
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                },
                [this] {
                    return std::abs(m_drive.GetHeading().value() - kHubRotationTarget) < kRotateToleranceDeg; //figure out if this is <= or >=
                },
                {&m_drive}

            ).ToPtr(),
            frc2::WaitCommand(units::second_t{5}).ToPtr()
        ),

        // Phase 3: Shoot
        frc2::cmd::Race(
            frc2::cmd::Sequence(
                frc2::InstantCommand([this] { m_shooter.Shoot(kShootRPM); }, {&m_shooter}).ToPtr(),
                frc2::WaitCommand(units::second_t{0.5}).ToPtr(),
                frc2::RunCommand([this] { m_shooter.RunCollector(); }, {&m_shooter}).ToPtr()
            ),
            frc2::WaitCommand(units::time::second_t{8}).ToPtr()
        ),

        // Phase 4: Stop shooting
        frc2::InstantCommand(
            [this] { m_shooter.Stop(); },
            {&m_shooter}
        ).ToPtr(),

        // Phase 5: Return to straight heading (180°) before strafe/climb
        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] {
                    
                },
                [this] {
                    m_drive.RotateToHeading(kClimbRotationTarget);
                },
                [this](bool) {
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                },
                [this] {
                    return std::abs(m_drive.GetHeading().value() - kHubRotationTarget) < kRotateToleranceDeg; 
                },
                {&m_drive}
            ).ToPtr(),
            frc2::WaitCommand(units::second_t{AutonomousRoutine::kRotateTimeout_s}).ToPtr()
        ),

        // Strafe left 1.5 feet (or 2.5s timeout)
        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] {
                    m_autoPhase8StartX = m_drive.GetPose().X().value();
                    m_autoPhase8StartY = m_drive.GetPose().Y().value();
                    m_autoTargetHeading = m_drive.GetYawDegrees();
                },
                [this] {
                    double rotCorrection = 0.0;
                    if (kHeadingCorrectionEnabled) {
                        double headingError = m_autoTargetHeading - m_drive.GetYawDegrees();
                        rotCorrection = headingError * kHeadingCorrectionPGain;
                    }
                    m_drive.driveRobotRelative(
                        frc::ChassisSpeeds{0_mps, units::meters_per_second_t{kStrafeSpeed}, units::radians_per_second_t{rotCorrection}}
                    );
                },
                [this](bool) {
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                },
                [this] {
                    double dx = m_drive.GetPose().X().value() - m_autoPhase8StartX;
                    double dy = m_drive.GetPose().Y().value() - m_autoPhase8StartY;
                    return std::sqrt(dx*dx + dy*dy) >= kStrafeDistance_ft * 0.3048;
                },
                {&m_drive}
            ).ToPtr(),
            frc2::WaitCommand(units::second_t{kStrafeTimeout_s}).ToPtr()
        )

        // Phase 9: Back up until limit switch triggered
        /*frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] { m_autoTargetHeading = m_drive.GetYawDegrees(); },
                [this] {
                    double rotCorrection = 0.0;
                    if (kHeadingCorrectionEnabled) {
                        double headingError = m_autoTargetHeading - m_drive.GetYawDegrees();
                        rotCorrection = headingError * kHeadingCorrectionPGain;
                    }
                    m_drive.driveRobotRelative(
                        frc::ChassisSpeeds{units::meters_per_second_t{-kBackupSpeed}, 0_mps, units::radians_per_second_t{rotCorrection}}
                    );
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
        frc2::cmd::Race(
        frc2::RunCommand(
            [this] { m_climber.Reverse(); }, // Lower to position 0
            {&m_climber}
        ).ToPtr(),
        frc2::WaitCommand(units::second_t{3_s}).ToPtr()
        )*/
    );
}

frc2::CommandPtr RobotContainer::GetOverBumpAuto(){
    using namespace AutonomousRoutine::OverBump;
    return frc2::cmd::Sequence(
         frc2::InstantCommand([this] { m_drive.ZeroHeading(); m_drive.ResetOdometry(frc::Pose2d{}); }, {&m_drive}).ToPtr(),

        frc2::WaitCommand(units::time::second_t{0.2}).ToPtr(), //give pigeon time to stabilize after reset

        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] {
                    //onInit
                    ConfigureAlliance();
                    m_autoTargetHeading = m_drive.GetYawDegrees();
                },
                [this] {
                    //onExec
                    m_drive.driveRobotRelative(
                        frc::ChassisSpeeds{0_mps, units::meters_per_second_t{kDriveSpeed}}
                    );
                },
                [this](bool) {
                    //onEnd
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                },
                [this] {
                    //isFinished?
                    double dist = m_drive.GetPose().Translation().Norm().value();
                    return dist >= kDriveDistanceOverBump * 0.3048;
                },
                {&m_drive}
            ).ToPtr(),
            frc2::WaitCommand(units::second_t{kSafetyTimeout}).ToPtr()
        ), //Drive over the bump

        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] {
                    //onInit
                    ConfigureAlliance();
                    m_autoTargetHeading = m_drive.GetYawDegrees();
                },
                [this] {
                    //onExec
                    m_drive.driveRobotRelative(
                        frc::ChassisSpeeds{units::meters_per_second_t{-kDriveSpeed}, 0_mps}
                    );
                },
                [this](bool) {
                    //onEnd
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                },
                [this] {
                    //isFinished?
                    double dist = m_drive.GetPose().Translation().Norm().value();
                    return dist >= kDriveDistanceBackwards * 0.3048;
                },
                {&m_drive}
            ).ToPtr(),
            frc2::WaitCommand(units::second_t{kSafetyTimeout}).ToPtr()
        ), //Go backwards to get around the fuel

        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] {
                    //onInit
                    ConfigureAlliance();
                    m_autoTargetHeading = m_drive.GetYawDegrees();
                },
                [this] {
                    //onExec
                    m_drive.driveRobotRelative(
                        frc::ChassisSpeeds{0_mps, units::meters_per_second_t{kDriveSpeed}}
                    );
                },
                [this](bool) {
                    //onEnd
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                },
                [this] {
                    //isFinished?
                    double dist = m_drive.GetPose().Translation().Norm().value();
                    return dist >= kDriveDistanceRight * 0.3048;
                },
                {&m_drive}
            ).ToPtr(),
            frc2::WaitCommand(units::second_t{kSafetyTimeout}).ToPtr()
        ), //Go to the right

        frc2::RunCommand([this]{m_intake.LowerLifter();},{&m_intake}).WithTimeout(2_s), //Lower the intake

        frc2::InstantCommand([this]{m_intake.Run();},{&m_intake}).ToPtr(), //Start intaking

        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] {
                    //onInit
                    ConfigureAlliance();
                    m_autoTargetHeading = m_drive.GetYawDegrees();
                },
                [this] {
                    //onExec
                    m_drive.driveRobotRelative(
                        frc::ChassisSpeeds{units::meters_per_second_t{kDriveSpeed}, 0_mps}
                    );
                },
                [this](bool) {
                    //onEnd
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                },
                [this] {
                    //isFinished?
                    double dist = m_drive.GetPose().Translation().Norm().value();
                    return dist >= kDriveDistanceThroughFuel * 0.3048;
                },
                {&m_drive}
            ).ToPtr(),
            frc2::WaitCommand(units::second_t{kSafetyTimeout}).ToPtr()
        ), //Go through the fuel to pick it up

        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] {
                    //onInit
                    ConfigureAlliance();
                    m_autoTargetHeading = m_drive.GetYawDegrees();
                },
                [this] {
                    //onExec
                    m_drive.driveRobotRelative(
                        frc::ChassisSpeeds{units::meters_per_second_t{-kDriveSpeed}, 0_mps}
                    );
                },
                [this](bool) {
                    //onEnd
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                },
                [this] {
                    //isFinished?
                    double dist = m_drive.GetPose().Translation().Norm().value();
                    return dist >= kDriveDistanceBack1 * 0.3048;
                },
                {&m_drive}
            ).ToPtr(),
            frc2::WaitCommand(units::second_t{kSafetyTimeout}).ToPtr()
        ), //Go back to line up with the bump

        frc2::InstantCommand([this]{m_intake.Stop();},{&m_intake}).ToPtr(), //Stop the intake

        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] {
                    //onInit
                    ConfigureAlliance();
                    m_autoTargetHeading = m_drive.GetYawDegrees();
                },
                [this] {
                    //onExec
                    m_drive.driveRobotRelative(
                        frc::ChassisSpeeds{0_mps, units::meters_per_second_t{-kDriveSpeed}}
                    );
                },
                [this](bool) {
                    //onEnd
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                },
                [this] {
                    //isFinished?
                    double dist = m_drive.GetPose().Translation().Norm().value();
                    return dist >= kDriveDistanceBack2 * 0.3048;
                },
                {&m_drive}
            ).ToPtr(),
            frc2::WaitCommand(units::second_t{kSafetyTimeout}).ToPtr()
        ), //Go back over the bump

        frc2::RunCommand([this]{m_drive.RotateToHeading(kShootHeading);},{&m_drive}).WithTimeout(2_s),

        frc2::InstantCommand([this]{m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps});},{&m_drive}).ToPtr(),

        frc2::RunCommand([this]{m_shooter.Shoot(kShootRPM);},{&m_shooter}).ToPtr()

    );
}