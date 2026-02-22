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
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>
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
  // Configure the button bindings
    ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
    auto team = frc::DriverStation::GetAlliance();
    if(team.value() == frc::DriverStation::Alliance::kRed){
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
            auto team = frc::DriverStation::GetAlliance(); // Otherwise sets lights to Alliance color.
            if(team.value() == frc::DriverStation::Alliance::kRed){ m_LEDs.TurnOnLEDs(1.0f, 0.0f, 0.0f); }
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

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
    return PathPlannerAuto("Test").ToPtr();
      // Set up config for trajectory
  /*frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
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
          [this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); }, {}));*/
}
