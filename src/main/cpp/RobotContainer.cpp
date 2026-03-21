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
    m_chooser.AddOption("rightClimb", "rightClimb");
    m_chooser.AddOption("overBump", "overBump");
    m_chooser.AddOption("overBumpLeftSide", "overBumpLeftSide");
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
            // TODO: SetPriorityTag is called every 20ms while bumper is held, writing to NetworkTables each cycle even though the value never changes. Move this to a JoystickButton.OnTrue() binding so it only fires once when the button is first pressed.
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


    // TODO: Distance-based RPM formula is commented out (old turret values). All branches currently call Shoot() with default RPM.
    // Once new distance-to-RPM values are collected for the wide shooter, re-enable the formula and update the distance thresholds and RPM calculation.
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

    // TODO: Add SmartDashboard::PutBoolean("Field Relative", fieldRelative) so drivers can see which mode they're in — currently no feedback if toggled accidentally
    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kStart).OnTrue
    (new frc2::InstantCommand([this] {fieldRelative = !fieldRelative;}));

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kBack).OnTrue
    (new frc2::InstantCommand([this] {m_drive.ZeroHeading();},{&m_drive}));

    frc2::JoystickButton(&m_coDriverController, frc::XboxController::Button::kY).OnTrue(new frc2::InstantCommand([this]{
        m_shooter.ReverseCollector();
        m_shooter.ReverseFeeder();
    },{&m_shooter})).OnFalse(new frc2::InstantCommand([this]{m_shooter.StopCollector(); m_shooter.StopFeeder();},{&m_shooter}));

    // TODO: Missing {&m_intake} subsystem requirement — default command will call Stop() 20ms later, making this button non-functional. Add {&m_intake} to both OnTrue and OnFalse InstantCommands
    frc2::JoystickButton(&m_coDriverController, frc::XboxController::Button::kA).OnTrue(new frc2::InstantCommand([this]{
        m_intake.Run(); //Reverse intake
    }))
    .OnFalse(new frc2::InstantCommand([this]{
        m_intake.Stop();
    }));

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
    if(selected == "rightClimb"){
        return GetRightSideClimbAuto();
    }
    if(selected == "overBump"){
        return GetOverBumpAuto();
    }
    if(selected == "overBumpLeftSide"){
        return GetOverBumpAutoLeftSide();
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

        // TODO: Climber code is commented out (mechanical changeover in progress)
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
                    // TODO: kDriveDistance1_ft is -12, so threshold is -3.66 — Norm() is always positive, so this is never true. Robot drives until timeout. Use std::abs(kDriveDistance1_ft) or make the constant positive and use >=
                    double dist = m_drive.GetPose().Translation().Norm().value();
                    return dist <= kDriveDistance1_ft * 0.3048;
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
                    // TODO: >= means if robot overshoots and PID corrects back below target, condition goes false and phase never finishes (runs until timeout). Use std::abs(heading - kHubRotationTarget) < kRotateToleranceDeg instead
                    return m_drive.GetHeading().value() >= kHubRotationTarget; //figure out if this is <= or >=
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
                    // TODO: Same issue as hub rotation — use std::abs(heading - kClimbRotationTarget) < kRotateToleranceDeg to handle overshoot
                    return m_drive.GetHeading().value() >= kClimbRotationTarget; //figure out if this is <= or >=
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

frc2::CommandPtr RobotContainer::GetRightSideClimbAuto(){
    // TODO: This auto never calls ConfigureAlliance() — m_isRedAlliance and camera priority tag won't be set. Other autos (ShootClimb, OverBump) all call it. Add ConfigureAlliance() in the first phase init if camera features are needed.
    using namespace AutonomousRoutine::RightBumpShootClimb;
    return frc2::cmd::Sequence(

        //make sure pigeon is fine
        frc2::InstantCommand([this] { m_drive.ZeroHeading(); m_drive.ResetOdometry(frc::Pose2d{}); }, {&m_drive}).ToPtr(),

        //Wait to make sure pigeon is fine
        frc2::WaitCommand(units::time::second_t{0.5}).ToPtr(),

        // TODO: kDrive1Speed is 0.1 m/s — to cover 9ft (2.74m) takes ~27s, but kDriveTimeout_s is 6s. Robot will only travel ~0.6m before timeout. Increase speed or timeout.
        //Go 11 ft
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
                        frc::ChassisSpeeds{units::meters_per_second_t{-kDrive1Speed}, 0_mps, units::radians_per_second_t{rotCorrection}}
                    );
                    m_climber.Run();
                },
                [this](bool) {
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                },
                [this] {
                    double dist = m_drive.GetPose().Translation().Norm().value();
                    return dist >= kDriveDistance1_ft * 0.3048;
                },
                {&m_drive,&m_climber}
            ).ToPtr(),
            frc2::WaitCommand(units::second_t{kDriveTimeout_s}).ToPtr()
        ),

        frc2::WaitCommand(units::second_t{0.25}).ToPtr(),

        // Phase 8: Strafe left 1.5 feet (or 2.5s timeout)
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
        ),
        

        // Back up until limit switch triggered
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

        frc2::WaitCommand(units::second_t{0.25}).ToPtr(),

        //Lower climber to climb
        frc2::cmd::Race(
        frc2::RunCommand(
            [this] { m_climber.Reverse(); },
            {&m_climber}
        ).ToPtr(),
        frc2::WaitCommand(units::second_t{3_s}).ToPtr()
        )

    );
}

frc2::CommandPtr RobotContainer::GetOverBumpAuto(){
    using namespace AutonomousRoutine::OverBump;

    return frc2::cmd::Sequence(
        frc2::InstantCommand([this] { m_drive.SetHeading(180.0); m_drive.ResetOdometry(frc::Pose2d{}); }, {&m_drive}).ToPtr(),

        //Wait to make sure pigeon is fine
        frc2::WaitCommand(units::time::second_t{0.2}).ToPtr(),

        // Rotate robot to shooting heading offset
        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] {
                    m_autoTargetHeading = 180.0 + AutonomousRoutine::OverBump::kShootHeadingOffset;
                },
                [this] {
                    double headingError = m_autoTargetHeading - m_drive.GetYawDegrees();
                    double rotSpeed = std::clamp(headingError * AutonomousRoutine::kRotatePGain, -0.5, 0.5);
                    m_drive.driveRobotRelative(
                        frc::ChassisSpeeds{0_mps, 0_mps, units::radians_per_second_t{rotSpeed}}
                    );
                },
                [this](bool) {
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                },
                [this] {
                    return std::abs(m_autoTargetHeading - m_drive.GetYawDegrees()) < AutonomousRoutine::kRotateToleranceDeg;
                },
                {&m_drive}
            ).ToPtr(),
            frc2::WaitCommand(units::time::second_t{1.0}).ToPtr()
        ),

        frc2::cmd::Race(
            frc2::cmd::Sequence(
                frc2::InstantCommand([this] { m_shooter.Shoot(kShootRPM); }, {&m_shooter}).ToPtr(),
                frc2::WaitCommand(units::second_t{0.2}).ToPtr(),
                frc2::RunCommand([this] { m_shooter.RunCollector(); }, {&m_shooter}).ToPtr()
            ),
            frc2::WaitCommand(units::time::second_t{3}).ToPtr()
        ),

        frc2::InstantCommand([this]{m_shooter.Stop();},{&m_shooter}).ToPtr(),

        // Rotate 180 degrees
        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] {
                    m_autoTargetHeading = m_drive.GetYawDegrees() + 180.0;
                },
                [this] {
                    double headingError = m_autoTargetHeading - m_drive.GetYawDegrees();
                    double rotSpeed = std::clamp(headingError * AutonomousRoutine::kRotatePGain, -0.5, 0.5);
                    m_drive.driveRobotRelative(
                        frc::ChassisSpeeds{0_mps, 0_mps, units::radians_per_second_t{rotSpeed}}
                    );
                },
                [this](bool) {
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                },
                [this] {
                    return std::abs(m_autoTargetHeading - m_drive.GetYawDegrees()) < AutonomousRoutine::kRotateToleranceDeg;
                },
                {&m_drive}
            ).ToPtr(),
            frc2::WaitCommand(units::second_t{AutonomousRoutine::kRotateTimeout_s}).ToPtr()
        ),

        frc2::cmd::Race(
            frc2::FunctionalCommand(
                // TODO: Missing ResetOdometry() — later drive phases reset before measuring distance but this one doesn't. Norm() includes drift from prior rotations, robot may stop short.
                [this] {
                    ConfigureAlliance();
                    m_autoTargetHeading = m_drive.GetYawDegrees();
                },
                [this] {
                    double rotCorrection = 0.0;
                    if (kHeadingCorrectionEnabled) {
                        double headingError = m_autoTargetHeading - m_drive.GetYawDegrees();
                        rotCorrection = headingError * kHeadingCorrectionPGain;
                    }
                    m_drive.driveRobotRelative(
                        frc::ChassisSpeeds{units::meters_per_second_t{kDriveSpeed}, 0_mps, units::radians_per_second_t{rotCorrection}}
                    );
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

        frc2::cmd::Race(
            frc2::RunCommand([this]{
                m_intake.LowerLifter();
            },{&m_intake}).ToPtr(),
            frc2::WaitCommand(units::second_t{kLowerIntakeTimeout}).ToPtr()
        ),

        frc2::InstantCommand([this]{m_intake.Stop();},{&m_intake}).ToPtr(),

        //frc2::WaitCommand(units::second_t{0.}).ToPtr(),

        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] {
                    //ConfigureAlliance();
                    // TODO: m_autoTargetHeading is not refreshed here — it still holds the value from a previous phase. If the robot drifted or rotated, heading correction will fight toward a stale heading. Uncomment the line below.
                    //m_autoTargetHeading = m_drive.GetYawDegrees();
                    m_drive.ResetOdometry(frc::Pose2d{});
                },
                [this] {
                    double rotCorrection = 0.0;
                    if (kHeadingCorrectionEnabled) {
                        double headingError = m_autoTargetHeading - m_drive.GetYawDegrees();
                        rotCorrection = headingError * kHeadingCorrectionPGain;
                    }
                    m_drive.driveRobotRelative(
                        frc::ChassisSpeeds{units::meters_per_second_t{kDriveSpeed2}, 0_mps, units::radians_per_second_t{rotCorrection}}
                    );
                    m_intake.Reverse();
                },
                [this](bool) {
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                },
                [this] {
                    double dist = m_drive.GetPose().Translation().Norm().value();
                    return dist >= kDriveDistance2_ft * 0.3048;
                },
                {&m_drive,&m_intake}
            ).ToPtr(),
            frc2::WaitCommand(units::second_t{kDriveTimeout2_s}).ToPtr()
        ),

        frc2::WaitCommand(units::second_t{0.25}).ToPtr(),

        frc2::InstantCommand([this]{m_intake.Stop();},{&m_intake}).ToPtr(),

        // TODO: kDriveSpeed3=0.25 m/s with kDriveTimeout3_s=5s means max travel is 1.25m (~4ft), but kDriveDistance3_ft=16.5ft (5.03m). Distance check is unreachable — will always end on timeout. Increase speed or timeout.
        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] {
                    //ConfigureAlliance();
                    // TODO: Same stale heading issue — uncomment m_autoTargetHeading = m_drive.GetYawDegrees()
                    //m_autoTargetHeading = m_drive.GetYawDegrees();
                    m_drive.ResetOdometry(frc::Pose2d{});
                },
                [this] {
                    double rotCorrection = 0.0;
                    if (kHeadingCorrectionEnabled) {
                        double headingError = m_autoTargetHeading - m_drive.GetYawDegrees();
                        rotCorrection = headingError * kHeadingCorrectionPGain;
                    }
                    m_drive.driveRobotRelative(
                        frc::ChassisSpeeds{units::meters_per_second_t{-kDriveSpeed3}, 0_mps, units::radians_per_second_t{rotCorrection}}
                    );
                },
                [this](bool) {
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                },
                [this] {
                    double dist = m_drive.GetPose().Translation().Norm().value();
                    return dist >= kDriveDistance3_ft * 0.3048;
                },
                {&m_drive,&m_intake}
            ).ToPtr(),
            frc2::WaitCommand(units::second_t{kDriveTimeout3_s}).ToPtr()
        ),

        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] {
                    m_autoTargetHeading = m_drive.GetYawDegrees() + 180.0;
                },
                [this] {
                    double headingError = m_autoTargetHeading - m_drive.GetYawDegrees();
                    double rotSpeed = std::clamp(headingError * AutonomousRoutine::kRotatePGain, -0.5, 0.5);
                    m_drive.driveRobotRelative(
                        frc::ChassisSpeeds{0_mps, 0_mps, units::radians_per_second_t{rotSpeed}}
                    );
                },
                [this](bool) {
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                },
                [this] {
                    return std::abs(m_autoTargetHeading - m_drive.GetYawDegrees()) < AutonomousRoutine::kRotateToleranceDeg;
                },
                {&m_drive}
            ).ToPtr(),
            frc2::WaitCommand(units::second_t{AutonomousRoutine::kRotateTimeout_s}).ToPtr()
        ),

        // TODO: Right-side OverBump does NOT have the extra kSecondShootHeadingOffset rotation that the left-side does before the second shot. If the right side also needs a heading offset, add the same rotation phase here.

        // TODO: Shoot() and RunCollector() are called simultaneously — balls feed before flywheels reach target RPM, causing weak shots. Add a 0.2-0.5s WaitCommand between Shoot() and RunCollector() like the first shot phase does.
        frc2::RunCommand([this]{m_shooter.Shoot(kShootRPM); m_shooter.RunCollector();},{&m_shooter}).WithTimeout(units::second_t{2}),

        frc2::RunCommand([this]{m_intake.RaiseLifter();},{&m_intake}).WithTimeout(units::second_t{2}),

        // TODO: This calls Stop() every 20ms for 3 seconds doing nothing — wastes 3s of auto time. Replace with an InstantCommand.
        frc2::RunCommand([this]{m_intake.Stop();},{&m_intake}).ToPtr().WithTimeout(units::second_t{3}),

        frc2::InstantCommand([this]{m_shooter.Stop();},{&m_shooter}).ToPtr()


    );
}

// TODO: This is nearly identical to GetOverBumpAuto() (~200 lines) — only the heading offset differs
// (kShootHeadingOffsetLeftSide vs kShootHeadingOffset). Refactor both into a single helper method
// that takes the heading offset as a parameter, e.g. GetOverBumpAuto(double headingOffset), to
// eliminate duplication and make both routines easier to maintain.
frc2::CommandPtr RobotContainer::GetOverBumpAutoLeftSide(){
    using namespace AutonomousRoutine::OverBump;

    return frc2::cmd::Sequence(
        frc2::InstantCommand([this] { m_drive.SetHeading(180.0); m_drive.ResetOdometry(frc::Pose2d{}); }, {&m_drive}).ToPtr(),

        //Wait to make sure pigeon is fine
        frc2::WaitCommand(units::time::second_t{0.2}).ToPtr(),

        // Rotate robot to shooting heading offset (left side)
        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] {
                    m_autoTargetHeading = 180.0 + AutonomousRoutine::OverBump::kShootHeadingOffsetLeftSide;
                },
                [this] {
                    double headingError = m_autoTargetHeading - m_drive.GetYawDegrees();
                    double rotSpeed = std::clamp(headingError * AutonomousRoutine::kRotatePGain, -0.5, 0.5);
                    m_drive.driveRobotRelative(
                        frc::ChassisSpeeds{0_mps, 0_mps, units::radians_per_second_t{rotSpeed}}
                    );
                },
                [this](bool) {
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                },
                [this] {
                    return std::abs(m_autoTargetHeading - m_drive.GetYawDegrees()) < AutonomousRoutine::kRotateToleranceDeg;
                },
                {&m_drive}
            ).ToPtr(),
            frc2::WaitCommand(units::time::second_t{1.0}).ToPtr()
        ),

        frc2::cmd::Race(
            frc2::cmd::Sequence(
                frc2::InstantCommand([this] { m_shooter.Shoot(kShootRPM); }, {&m_shooter}).ToPtr(),
                frc2::WaitCommand(units::second_t{0.2}).ToPtr(),
                frc2::RunCommand([this] { m_shooter.RunCollector(); }, {&m_shooter}).ToPtr()
            ),
            frc2::WaitCommand(units::time::second_t{3}).ToPtr()
        ),

        frc2::InstantCommand([this]{m_shooter.Stop();},{&m_shooter}).ToPtr(),

        // Rotate 180 degrees
        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] {
                    m_autoTargetHeading = m_drive.GetYawDegrees() + 180.0;
                },
                [this] {
                    double headingError = m_autoTargetHeading - m_drive.GetYawDegrees();
                    double rotSpeed = std::clamp(headingError * AutonomousRoutine::kRotatePGain, -0.5, 0.5);
                    m_drive.driveRobotRelative(
                        frc::ChassisSpeeds{0_mps, 0_mps, units::radians_per_second_t{rotSpeed}}
                    );
                },
                [this](bool) {
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                },
                [this] {
                    return std::abs(m_autoTargetHeading - m_drive.GetYawDegrees()) < AutonomousRoutine::kRotateToleranceDeg;
                },
                {&m_drive}
            ).ToPtr(),
            frc2::WaitCommand(units::second_t{AutonomousRoutine::kRotateTimeout_s}).ToPtr()
        ),

        frc2::cmd::Race(
            frc2::FunctionalCommand(
                // TODO: Missing ResetOdometry() — later drive phases reset before measuring distance but this one doesn't. Norm() includes drift from prior rotations, robot may stop short.
                [this] {
                    ConfigureAlliance();
                    m_autoTargetHeading = m_drive.GetYawDegrees();
                },
                [this] {
                    double rotCorrection = 0.0;
                    if (kHeadingCorrectionEnabled) {
                        double headingError = m_autoTargetHeading - m_drive.GetYawDegrees();
                        rotCorrection = headingError * kHeadingCorrectionPGain;
                    }
                    m_drive.driveRobotRelative(
                        frc::ChassisSpeeds{units::meters_per_second_t{kDriveSpeed}, 0_mps, units::radians_per_second_t{rotCorrection}}
                    );
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

        frc2::cmd::Race(
            frc2::RunCommand([this]{
                m_intake.LowerLifter();
            },{&m_intake}).ToPtr(),
            frc2::WaitCommand(units::second_t{kLowerIntakeTimeout}).ToPtr()
        ),

        frc2::InstantCommand([this]{m_intake.Stop();},{&m_intake}).ToPtr(),

        //frc2::WaitCommand(units::second_t{0.}).ToPtr(),

        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] {
                    //ConfigureAlliance();
                    // TODO: m_autoTargetHeading is not refreshed here — it still holds the value from a previous phase. If the robot drifted or rotated, heading correction will fight toward a stale heading. Uncomment the line below.
                    //m_autoTargetHeading = m_drive.GetYawDegrees();
                    m_drive.ResetOdometry(frc::Pose2d{});
                },
                [this] {
                    double rotCorrection = 0.0;
                    if (kHeadingCorrectionEnabled) {
                        double headingError = m_autoTargetHeading - m_drive.GetYawDegrees();
                        rotCorrection = headingError * kHeadingCorrectionPGain;
                    }
                    m_drive.driveRobotRelative(
                        frc::ChassisSpeeds{units::meters_per_second_t{kDriveSpeed2}, 0_mps, units::radians_per_second_t{rotCorrection}}
                    );
                    m_intake.Reverse();
                },
                [this](bool) {
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                },
                [this] {
                    double dist = m_drive.GetPose().Translation().Norm().value();
                    return dist >= kDriveDistance2_ft * 0.3048;
                },
                {&m_drive,&m_intake}
            ).ToPtr(),
            frc2::WaitCommand(units::second_t{kDriveTimeout2_s}).ToPtr()
        ),

        frc2::WaitCommand(units::second_t{0.25}).ToPtr(),

        frc2::InstantCommand([this]{m_intake.Stop();},{&m_intake}).ToPtr(),

        // TODO: Same speed/distance mismatch as right-side — kDriveSpeed3=0.25 m/s, kDriveTimeout3_s=5s, max travel 1.25m but target is 16.5ft. Will always hit timeout.
        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] {
                    //ConfigureAlliance();
                    // TODO: Same stale heading issue — uncomment m_autoTargetHeading = m_drive.GetYawDegrees()
                    //m_autoTargetHeading = m_drive.GetYawDegrees();
                    m_drive.ResetOdometry(frc::Pose2d{});
                },
                [this] {
                    double rotCorrection = 0.0;
                    if (kHeadingCorrectionEnabled) {
                        double headingError = m_autoTargetHeading - m_drive.GetYawDegrees();
                        rotCorrection = headingError * kHeadingCorrectionPGain;
                    }
                    m_drive.driveRobotRelative(
                        frc::ChassisSpeeds{units::meters_per_second_t{-kDriveSpeed3}, 0_mps, units::radians_per_second_t{rotCorrection}}
                    );
                },
                [this](bool) {
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                },
                [this] {
                    double dist = m_drive.GetPose().Translation().Norm().value();
                    return dist >= kDriveDistance3_ft * 0.3048;
                },
                {&m_drive,&m_intake}
            ).ToPtr(),
            frc2::WaitCommand(units::second_t{kDriveTimeout3_s}).ToPtr()
        ),

        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] {
                    m_autoTargetHeading = m_drive.GetYawDegrees() + 180.0;
                },
                [this] {
                    double headingError = m_autoTargetHeading - m_drive.GetYawDegrees();
                    double rotSpeed = std::clamp(headingError * AutonomousRoutine::kRotatePGain, -0.5, 0.5);
                    m_drive.driveRobotRelative(
                        frc::ChassisSpeeds{0_mps, 0_mps, units::radians_per_second_t{rotSpeed}}
                    );
                },
                [this](bool) {
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                },
                [this] {
                    return std::abs(m_autoTargetHeading - m_drive.GetYawDegrees()) < AutonomousRoutine::kRotateToleranceDeg;
                },
                {&m_drive}
            ).ToPtr(),
            frc2::WaitCommand(units::second_t{AutonomousRoutine::kRotateTimeout_s}).ToPtr()
        ),

        // Rotate to second shot heading offset
        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] {
                    m_autoTargetHeading = m_drive.GetYawDegrees() + AutonomousRoutine::OverBump::kSecondShootHeadingOffset;
                },
                [this] {
                    double headingError = m_autoTargetHeading - m_drive.GetYawDegrees();
                    double rotSpeed = std::clamp(headingError * AutonomousRoutine::kRotatePGain, -0.5, 0.5);
                    m_drive.driveRobotRelative(
                        frc::ChassisSpeeds{0_mps, 0_mps, units::radians_per_second_t{rotSpeed}}
                    );
                },
                [this](bool) {
                    m_drive.driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
                },
                [this] {
                    return std::abs(m_autoTargetHeading - m_drive.GetYawDegrees()) < AutonomousRoutine::kRotateToleranceDeg;
                },
                {&m_drive}
            ).ToPtr(),
            frc2::WaitCommand(units::second_t{1.0}).ToPtr()
        ),

        // TODO: Same as right-side — Shoot() and RunCollector() called simultaneously, balls feed before flywheels at speed. Add a spin-up delay.
        frc2::RunCommand([this]{m_shooter.Shoot(kShootRPM); m_shooter.RunCollector();},{&m_shooter}).WithTimeout(units::second_t{2}),

        frc2::RunCommand([this]{m_intake.RaiseLifter();},{&m_intake}).WithTimeout(units::second_t{2}),

        // TODO: Same as right-side — calls Stop() every 20ms for 3s doing nothing, wastes auto time. Replace with InstantCommand.
        frc2::RunCommand([this]{m_intake.Stop();},{&m_intake}).ToPtr().WithTimeout(units::second_t{3}),

        frc2::InstantCommand([this]{m_shooter.Stop();},{&m_shooter}).ToPtr()


    );
}

