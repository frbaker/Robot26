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
//#include <pathplanner/lib/commands/PathPlannerAuto.h>
//#include <pathplanner/lib/auto/NamedCommands.h>

#include <utility>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"


using namespace DriveConstants;
//using namespace pathplanner;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

    /*NamedCommands::registerCommand("Shoot", std::move(m_shooter.ShootAuto()));
    NamedCommands::registerCommand("StopShooting", std::move(m_shooter.StopAuto()));
    NamedCommands::registerCommand("ClimberUp", std::move(m_climber.UpAuto()));
    NamedCommands::registerCommand("ClimberDown", std::move(m_climber.DownAuto()));*/

  // Auto chooser
    m_chooser.SetDefaultOption("shootClimb", "shootClimb");
    m_chooser.AddOption("rightClimb", "rightClimb");
    m_chooser.AddOption("overBump", "overBump");
    m_chooser.AddOption("overBumpLeftSide", "overBumpLeftSide");
    frc::SmartDashboard::PutData("Auto Selector", &m_chooser);

  // Configure the button bindings
    ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
    m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        auto xSpeed = -units::meters_per_second_t{frc::ApplyDeadband(
            m_driverController.GetLeftY(), OIConstants::kDriveDeadband)};
        auto ySpeed = -units::meters_per_second_t{frc::ApplyDeadband(
            m_driverController.GetLeftX(), OIConstants::kDriveDeadband)};

        units::radians_per_second_t rot{0.0};

        if (m_driverController.GetAButton() || m_driverController.GetBButton()) {
            if (!m_teleSpinActive) {
                double direction = m_driverController.GetAButton() ? 180.0 : -180.0;
                m_teleSpinTarget = m_drive.GetYawDegrees() + direction;
                m_teleSpinActive = true;
            }
            double headingError = m_teleSpinTarget - m_drive.GetYawDegrees();
            double rotOverride = std::clamp(headingError * OIConstants::kSpinPGain,
                                            -OIConstants::kSpinClamp, OIConstants::kSpinClamp);
            rot = -units::radians_per_second_t{rotOverride};
        } else {
            m_teleSpinActive = false;
            rot = -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController.GetRightX(), OIConstants::kDriveDeadband)};
        }

        m_drive.Drive(xSpeed, ySpeed, rot, fieldRelative);
      },
      {&m_drive}));

    m_LEDs.SetDefaultCommand(frc2::RunCommand([this]{
        if((m_camera.GetDetection() == true) ){
            double distance = m_camera.GetDistance();
                if((distance >= 5) && (distance <= 15)){
                    if(((m_camera.GetTagId()==AprilTags::Hub::kBlueCenter) || m_camera.GetTagId()==AprilTags::Hub::kRedCenter)){
                        m_coDriverController.SetRumble(frc::GenericHID::kRightRumble, 0.015);
                        m_LEDs.TurnOnLEDs(0.0f, 0.5f, 0.0f); // If the camera sees an AprilTag, sets lights to green
                    }
                }
        } else{
            m_coDriverController.SetRumble(frc::GenericHID::kBothRumble, 0.0);
            if(m_isRedAlliance){ m_LEDs.TurnOnLEDs(1.0f, 0.0f, 0.0f); }
            else{ m_LEDs.TurnOnLEDs(0.0f, 0.0f, 1.0f); }
            //m_LEDs.GO(0,0,0);
        }
    },{&m_LEDs}));

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
        if(m_coDriverController.GetLeftBumperButton()){
            //std::cout << "lef bumper haha" << std::endl;
            m_camera.SetPriorityTag(2);
            if(m_camera.GetDetection()){
                 //std::cout << "lef bumper goot" << std::endl;
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

    /*4ft 2750rpm
    8ft 3275rpm
    12ft 3800rpm

    131 per ft

    x = 12-ft
    y = x*131
    z = 3800-y

    power = z

    over bump auto 2900rpm
    left side turret 7.8turret*/

    //review

    frc2::JoystickButton(&m_coDriverController, frc::XboxController::Button::kRightBumper).OnTrue(
        new frc2::InstantCommand([this] {
            if(m_camera.GetDetection()){
                double distance = m_camera.GetDistance();
                if((distance > 4) && (distance < 15)){
                    m_shooter.Shoot(3800-((12-distance)*131));
                    frc2::WaitCommand(units::second_t{0.2}).ToPtr();
                    m_shooter.RunCollector();
                }
                
            }
            else{
                m_shooter.Shoot();
                frc2::WaitCommand(units::second_t{0.2}).ToPtr();
                m_shooter.RunCollector();
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
    m_turret.SetSpeed(0);
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

        frc2::WaitCommand(units::time::second_t{0.5}).ToPtr(),

        // Phase 1: Drive forward 12 inches
        frc2::cmd::Race(
            frc2::FunctionalCommand(
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

        // Phase 2: Aim turret at AprilTag
        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] { 
                    m_autoTurretStartPos = m_turret.GetPosition();
                },
                [this] {
                    if (m_camera.GetDetection()) {
                        m_turret.PointAtAprilTag(-m_camera.GetYaw());
                    }
                    else{
                        m_turret.SetSpeed(0);
                    }
                },
                [this](bool) { m_turret.SetSpeed(0); },
                [this] {
                    return m_camera.GetDetection() &&
                           std::abs(m_camera.GetYaw()) < kTurretAimYawTolerance;
                },
                {&m_turret, &m_camera}
            ).ToPtr(),
            frc2::WaitCommand(units::second_t{kRotateTimeout_s}).ToPtr()
        ),

        // Phase 3: Shoot for 7 seconds while turret keeps tracking
        frc2::cmd::Race(
            frc2::RunCommand([this] {
                m_shooter.Shoot(kShootRPM);
                frc2::WaitCommand(units::second_t{0.2});
                m_shooter.RunCollector();
            }, {&m_shooter}).ToPtr(),
            frc2::FunctionalCommand(
                [this] {},
                [this] {
                    if (m_camera.GetDetection()) {
                        m_turret.PointAtAprilTag(-m_camera.GetYaw());
                    }
                    else{
                        m_turret.SetSpeed(0);
                    }
                },
                [this](bool) { m_turret.SetSpeed(0); },
                [this] { return false; },
                {&m_turret, &m_camera}
            ).ToPtr(),
            frc2::WaitCommand(units::time::second_t{7}).ToPtr()
        ),

        // Phase 4: Stop shooting
        frc2::InstantCommand(
            [this] { m_shooter.Stop(); },
            {&m_shooter}
        ).ToPtr(),

        frc2::cmd::Race(
            frc2::RunCommand([this]{m_climber.Run();},{&m_climber}).ToPtr(),
            frc2::WaitCommand(units::second_t{2}).ToPtr()
        ),
        
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
        )
    );
}

frc2::CommandPtr RobotContainer::GetRightSideClimbAuto(){

    using namespace AutonomousRoutine::RightBumpShootClimb;
    return frc2::cmd::Sequence(

        //make sure pigeon is fine
        frc2::InstantCommand([this] { m_drive.ZeroHeading(); m_drive.ResetOdometry(frc::Pose2d{}); }, {&m_drive}).ToPtr(),

        //Wait to make sure pigeon is fine
        frc2::WaitCommand(units::time::second_t{0.5}).ToPtr(),

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
        frc2::InstantCommand([this] { m_drive.ZeroHeading(); m_drive.ResetOdometry(frc::Pose2d{}); }, {&m_drive}).ToPtr(),

        //Wait to make sure pigeon is fine
        frc2::WaitCommand(units::time::second_t{0.2}).ToPtr(),

        frc2::cmd::Race(
            frc2::RunCommand([this]{m_turret.SetPoint(-5.65);},{&m_turret}).ToPtr(),
            frc2::WaitCommand(units::time::second_t{0.2}).ToPtr()
        ),

        frc2::cmd::Race(

            frc2::RunCommand([this] {
                m_shooter.Shoot(kShootRPM);
                frc2::WaitCommand(units::second_t{0.2});
                m_shooter.RunCollector();
            }, {&m_shooter}).ToPtr(),

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

        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] {
                    //ConfigureAlliance();
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

        //frc2::InstantCommand([this]{m_turret.SetPoint(6.28);},{&m_turret}).ToPtr(),

        frc2::RunCommand([this]{m_shooter.Shoot(kShootRPM); m_shooter.RunCollector();},{&m_shooter}).WithTimeout(units::second_t{2}),

        frc2::RunCommand([this]{m_intake.RaiseLifter();},{&m_intake}).WithTimeout(units::second_t{2}),

        frc2::RunCommand([this]{m_intake.Stop();},{&m_intake}).ToPtr().WithTimeout(units::second_t{3}),

        frc2::InstantCommand([this]{m_shooter.Stop();},{&m_shooter}).ToPtr()


    );
}

frc2::CommandPtr RobotContainer::GetOverBumpAutoLeftSide(){
    using namespace AutonomousRoutine::OverBump;
    
    return frc2::cmd::Sequence(
        frc2::InstantCommand([this] { m_drive.ZeroHeading(); m_drive.ResetOdometry(frc::Pose2d{}); }, {&m_drive}).ToPtr(),

        //Wait to make sure pigeon is fine
        frc2::WaitCommand(units::time::second_t{0.2}).ToPtr(),

        frc2::cmd::Race(
            frc2::RunCommand([this]{m_turret.SetPoint(7.8);},{&m_turret}).ToPtr(),
            frc2::WaitCommand(units::time::second_t{0.2}).ToPtr()
        ),

        frc2::cmd::Race(

            frc2::RunCommand([this] {
                m_shooter.Shoot(kShootRPM);
                frc2::WaitCommand(units::second_t{0.2});
                m_shooter.RunCollector();
            }, {&m_shooter}).ToPtr(),

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

        frc2::cmd::Race(
            frc2::FunctionalCommand(
                [this] {
                    //ConfigureAlliance();
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

        frc2::InstantCommand([this]{m_turret.SetPoint(6.28);},{&m_turret}).ToPtr(),

        frc2::RunCommand([this]{m_shooter.Shoot(kShootRPM); m_shooter.RunCollector();},{&m_shooter}).WithTimeout(units::second_t{2}),

        frc2::RunCommand([this]{m_intake.RaiseLifter();},{&m_intake}).WithTimeout(units::second_t{2}),

        frc2::RunCommand([this]{m_intake.Stop();},{&m_intake}).ToPtr().WithTimeout(units::second_t{3}),

        frc2::InstantCommand([this]{m_shooter.Stop();},{&m_shooter}).ToPtr()


    );
}

