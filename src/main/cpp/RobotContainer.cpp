// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/DriverStation.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/velocity.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

#include <utility>
#include <cmath>
#include <memory>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "commands/AutoCommands.h"

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
        if(m_camera.detection.Get() == true){
            m_LEDs.GO(0, 1, 0);
        } else{
            auto team = frc::DriverStation::GetAlliance();

            if(team.value() == frc::DriverStation::Alliance::kRed){ m_LEDs.GO(1, 0, 0); }
            else{ m_LEDs.GO(0, 0, 1); }
        }
    },{&m_camera, &m_LEDs}));

    // ========== PATHPLANNER CONFIGURATION ==========

    // Register named commands FIRST (before AutoBuilder configure)
    AutoCommands::RegisterCommands(&m_drive, &m_intake, &m_shooter, &m_turret, &m_camera, &m_climber);

    // Configure AutoBuilder for swerve drive
    pathplanner::AutoBuilder::configure(
        // Pose supplier - gets current robot pose from odometry
        [this]() { return m_drive.GetPose(); },

        // Pose reset consumer - resets odometry to given pose
        [this](frc::Pose2d pose) { m_drive.ResetOdometry(pose); },

        // ChassisSpeeds supplier - gets current robot-relative speeds
        [this]() { return m_drive.GetRobotRelativeSpeeds(); },

        // ChassisSpeeds consumer - drives robot with given speeds
        [this](frc::ChassisSpeeds speeds) { m_drive.DriveRobotRelative(speeds); },

        // Path follower controller (must be shared_ptr)
        std::make_shared<pathplanner::PPHolonomicDriveController>(
            // Translation PID constants
            pathplanner::PIDConstants(AutoConstants::kPXController, 0.0, 0.0),
            // Rotation PID constants
            pathplanner::PIDConstants(AutoConstants::kPThetaController, 0.0, 0.0)
        ),

        // Robot config (max speed, drive base radius, mass, MOI)
        pathplanner::RobotConfig::fromGUISettings(),

        // Should flip path for alliance (red vs blue)
        []() {
            auto alliance = frc::DriverStation::GetAlliance();
            return alliance.has_value() &&
                   alliance.value() == frc::DriverStation::Alliance::kRed;
        },

        // Drive subsystem requirement
        &m_drive
    );

    // Set up auto chooser
    // PathPlanner autos (recommended)
    m_chooser.SetDefaultOption("Depot Sweep", "PP:DepotSweep");
    m_chooser.AddOption("Outpost Dump", "PP:OutPostDump");
    m_chooser.AddOption("Neutral Zone Left", "PP:NeutralZoneLeft");
    m_chooser.AddOption("Neutral Zone Right", "PP:NeutralZoneRight");
    m_chooser.AddOption("Side Climb", "PP:SideClimb");
    // Legacy autos (fallback)
    m_chooser.AddOption("Legacy: Depot Sweep", "Legacy:DepotSweep");
    m_chooser.AddOption("Legacy: Outpost Dump", "Legacy:OutpostDump");
    m_chooser.AddOption("Legacy: Neutral Zone", "Legacy:NeutralZone");
    m_chooser.AddOption("Legacy: Side Climb", "Legacy:SideClimb");
    frc::SmartDashboard::PutData("Auto Chooser", &m_chooser);
}

void RobotContainer::ConfigureButtonBindings() {
    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kRightBumper).WhileTrue
    (new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kA).OnTrue(
        new frc2::InstantCommand([this] {m_shooter.Shoot();},{&m_shooter})).OnFalse(
            new frc2::InstantCommand([this] {m_shooter.Stop();}, {&m_shooter})
    );

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kStart).OnTrue
    (new frc2::InstantCommand([this] {fieldRelative = !fieldRelative;}));

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kLeftBumper).WhileTrue(new frc2::RunCommand([this]{
        m_turret.PointAtAprilTag(m_camera.yaw.Get());
    }, {&m_turret, &m_camera}));

    // Intake controls - B to run, X to reverse
    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kB).OnTrue(
        new frc2::InstantCommand([this] { m_intake.Run(); }, {&m_intake})).OnFalse(
            new frc2::InstantCommand([this] { m_intake.Stop(); }, {&m_intake})
    );

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kX).OnTrue(
        new frc2::InstantCommand([this] { m_intake.Reverse(); }, {&m_intake})).OnFalse(
            new frc2::InstantCommand([this] { m_intake.Stop(); }, {&m_intake})
    );

    // Climber controls - Y to climb up
    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kY).OnTrue(
        new frc2::InstantCommand([this] { m_climber.Climb(); }, {&m_climber})).OnFalse(
            new frc2::InstantCommand([this] { m_climber.Stop(); }, {&m_climber})
    );
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    std::string selected = m_chooser.GetSelected();

    // Check for PathPlanner autos (PP: prefix)
    if (selected.rfind("PP:", 0) == 0) {
        std::string autoName = selected.substr(3);  // Remove "PP:" prefix
        return pathplanner::PathPlannerAuto(autoName).ToPtr().Unwrap().release();
    }

    // Legacy autos (Legacy: prefix or direct match for backwards compatibility)
    if (selected == "Legacy:DepotSweep") {
        return DepotSweepAuto();
    } else if (selected == "Legacy:OutpostDump") {
        return OutpostDumpAuto();
    } else if (selected == "Legacy:NeutralZone") {
        return NeutralZoneAuto();
    } else if (selected == "Legacy:SideClimb") {
        return SideClimbAuto();
    }

    // Default - do nothing
    return new frc2::InstantCommand([] {});
}

// Helper to get AprilTag IDs based on alliance
int GetOutpostTag() {
    auto alliance = frc::DriverStation::GetAlliance();
    if (alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kRed) {
        return AprilTagConstants::Red::kOutpost;
    }
    return AprilTagConstants::Blue::kOutpost;
}

int GetLadderTag1() {
    auto alliance = frc::DriverStation::GetAlliance();
    if (alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kRed) {
        return AprilTagConstants::Red::kLadder1;
    }
    return AprilTagConstants::Blue::kLadder1;
}

int GetLadderTag2() {
    auto alliance = frc::DriverStation::GetAlliance();
    if (alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kRed) {
        return AprilTagConstants::Red::kLadder2;
    }
    return AprilTagConstants::Blue::kLadder2;
}

int GetHubAllianceTag1() {
    auto alliance = frc::DriverStation::GetAlliance();
    if (alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kRed) {
        return AprilTagConstants::Red::kHubAllianceSide1;
    }
    return AprilTagConstants::Blue::kHubAllianceSide1;
}

int GetHubAllianceTag2() {
    auto alliance = frc::DriverStation::GetAlliance();
    if (alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kRed) {
        return AprilTagConstants::Red::kHubAllianceSide2;
    }
    return AprilTagConstants::Blue::kHubAllianceSide2;
}

int GetHubNeutralTag1() {
    auto alliance = frc::DriverStation::GetAlliance();
    if (alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kRed) {
        return AprilTagConstants::Red::kHubNeutralSide1;
    }
    return AprilTagConstants::Blue::kHubNeutralSide1;
}

int GetHubNeutralTag2() {
    auto alliance = frc::DriverStation::GetAlliance();
    if (alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kRed) {
        return AprilTagConstants::Red::kHubNeutralSide2;
    }
    return AprilTagConstants::Blue::kHubNeutralSide2;
}

// Depot Sweep: Start near hub, drive to depot, collect fuel, return to hub, shoot
// Single cycle auto
frc2::Command* RobotContainer::DepotSweepAuto() {
    return new frc2::SequentialCommandGroup(
        // 1. Reset odometry at starting position
        frc2::InstantCommand([this] {
            m_drive.ResetOdometry(frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}});
        }, {&m_drive}),

        // 2. Drive backward toward depot (~15 feet / 4.57 meters)
        //    Turret tracks hub tag, use yaw to correct drivetrain heading
        frc2::FunctionalCommand(
            [this] {}, // Init
            [this] {
                // Get camera yaw to hub tag
                double yawCorrection = 0.0;
                int targetTag = GetHubAllianceTag1(); // Tag 9 for red, 25 for blue

                if (m_camera.detection.Get() && m_camera.tagId.Get() == targetTag) {
                    // Point turret at tag
                    m_turret.PointAtAprilTag(m_camera.yaw.Get());
                    // Use yaw to correct drivetrain rotation (negative because driving backward)
                    yawCorrection = -m_camera.yaw.Get() * 0.05; // Tune this gain
                }

                // Drive backward (negative X), use yaw for rotation correction
                m_drive.Drive(-1.0_mps, 0_mps, units::radians_per_second_t{yawCorrection}, false);
            },
            [this](bool) { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); }, // End - stop
            [this] {
                // End when we've traveled ~15 feet (4.57 meters)
                auto pose = m_drive.GetPose();
                double distance = std::sqrt(std::pow(pose.X().value(), 2) + std::pow(pose.Y().value(), 2));
                return distance >= 4.57; // 15 feet in meters
            },
            {&m_drive, &m_camera, &m_turret}
        ),

        // 3. At depot - run intake for 3 seconds to collect fuel
        frc2::InstantCommand([this] { m_intake.Run(); }, {&m_intake}),
        frc2::WaitCommand(3.0_s),
        frc2::InstantCommand([this] { m_intake.Stop(); }, {&m_intake}),

        // 4. Reset odometry for return trip
        frc2::InstantCommand([this] {
            m_drive.ResetOdometry(frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}});
        }, {&m_drive}),

        // 5. Drive toward hub (shooter side facing hub)
        //    Turret tracks hub tag, use yaw to correct heading
        frc2::FunctionalCommand(
            [this] {}, // Init
            [this] {
                double yawCorrection = 0.0;
                int targetTag = GetHubAllianceTag1(); // Tag 9 for red, 25 for blue

                if (m_camera.detection.Get() && m_camera.tagId.Get() == targetTag) {
                    m_turret.PointAtAprilTag(m_camera.yaw.Get());
                    yawCorrection = m_camera.yaw.Get() * 0.05; // Tune this gain
                }

                // Drive forward (positive X since shooter faces hub behind us)
                m_drive.Drive(1.0_mps, 0_mps, units::radians_per_second_t{yawCorrection}, false);
            },
            [this](bool) { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); }, // End - stop
            [this] {
                // End when we've traveled back ~15 feet or see tag close enough
                auto pose = m_drive.GetPose();
                double distance = std::sqrt(std::pow(pose.X().value(), 2) + std::pow(pose.Y().value(), 2));
                return distance >= 4.57; // 15 feet in meters
            },
            {&m_drive, &m_camera, &m_turret}
        ),

        // 6. Final turret alignment to hub tag
        frc2::FunctionalCommand(
            [this] {},
            [this] {
                int targetTag = GetHubAllianceTag1();
                if (m_camera.detection.Get() && m_camera.tagId.Get() == targetTag) {
                    m_turret.PointAtAprilTag(m_camera.yaw.Get());
                }
            },
            [this](bool) {},
            [this] {
                // End when turret is aligned (yaw close to 0)
                int targetTag = GetHubAllianceTag1();
                return m_camera.detection.Get() &&
                       m_camera.tagId.Get() == targetTag &&
                       std::abs(m_camera.yaw.Get()) < 2.0; // Within 2 degrees
            },
            {&m_camera, &m_turret}
        ),

        // 7. Shoot fuel into hub
        frc2::InstantCommand([this] { m_shooter.Shoot(); }, {&m_shooter}),
        frc2::WaitCommand(2.0_s),
        frc2::InstantCommand([this] { m_shooter.Stop(); }, {&m_shooter}),

        // 8. Stop everything
        frc2::InstantCommand([this] {
            m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false);
        }, {&m_drive})
    );
}

// Outpost Dump: Start near hub facing outpost, drive to outpost, receive fuel dump,
// return to hub and shoot
// Single cycle auto
frc2::Command* RobotContainer::OutpostDumpAuto() {
    return new frc2::SequentialCommandGroup(
        // 1. Reset odometry and store starting heading
        frc2::InstantCommand([this] {
            m_drive.ResetOdometry(frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}});
        }, {&m_drive}),

        // 2. Drive forward toward outpost (~15 feet) using odometry
        //    Robot facing outpost, shooter/camera facing hub behind
        frc2::FunctionalCommand(
            [this] {}, // Init
            [this] {
                // Drive forward toward outpost
                m_drive.Drive(1.0_mps, 0_mps, 0_rad_per_s, false);
            },
            [this](bool) { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); },
            [this] {
                // End when traveled ~12 feet (save 3 feet for vision approach)
                auto pose = m_drive.GetPose();
                double distance = std::sqrt(std::pow(pose.X().value(), 2) + std::pow(pose.Y().value(), 2));
                return distance >= 3.66; // ~12 feet in meters
            },
            {&m_drive}
        ),

        // 3. Vision-based final approach to outpost tag 13/29
        frc2::FunctionalCommand(
            [this] {}, // Init
            [this] {
                int targetTag = GetOutpostTag(); // Tag 13 for red, 29 for blue
                double yawCorrection = 0.0;

                if (m_camera.detection.Get() && m_camera.tagId.Get() == targetTag) {
                    // Use yaw to steer toward tag
                    yawCorrection = m_camera.yaw.Get() * 0.05;
                }

                // Drive forward with yaw correction
                m_drive.Drive(0.5_mps, 0_mps, units::radians_per_second_t{yawCorrection}, false);
            },
            [this](bool) { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); },
            [this] {
                // End when we see outpost tag and are close enough
                int targetTag = GetOutpostTag();
                return m_camera.detection.Get() &&
                       m_camera.tagId.Get() == targetTag &&
                       m_camera.distance.Get() < 5.0; // feet - close enough to start positioning
            },
            {&m_drive, &m_camera}
        ),

        // 4. Rotate chassis 90 degrees clockwise (right side faces wall)
        frc2::FunctionalCommand(
            [this] {
                m_drive.ResetOdometry(frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}});
            },
            [this] {
                // Rotate clockwise (negative rotation in WPILib convention)
                m_drive.Drive(0_mps, 0_mps, -1.5_rad_per_s, false);
            },
            [this](bool) { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); },
            [this] {
                // End when rotated ~90 degrees
                auto heading = m_drive.GetHeading();
                return heading.value() <= -85.0; // Close to -90 degrees
            },
            {&m_drive}
        ),

        // 5. Strafe right to position for dump (right side toward wall)
        frc2::FunctionalCommand(
            [this] {
                m_drive.ResetOdometry(frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}});
            },
            [this] {
                // Strafe right (negative Y in robot-relative)
                m_drive.Drive(0_mps, -0.5_mps, 0_rad_per_s, false);
            },
            [this](bool) { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); },
            [this] {
                // Strafe for ~2 feet (0.6 meters) - adjust as needed
                auto pose = m_drive.GetPose();
                return std::abs(pose.Y().value()) >= 0.6;
            },
            {&m_drive}
        ),

        // 6. Wait 3 seconds for human player to dump fuel
        frc2::WaitCommand(3.0_s),

        // 7. Strafe left to clear the outpost
        frc2::FunctionalCommand(
            [this] {
                m_drive.ResetOdometry(frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}});
            },
            [this] {
                // Strafe left (positive Y in robot-relative)
                m_drive.Drive(0_mps, 0.5_mps, 0_rad_per_s, false);
            },
            [this](bool) { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); },
            [this] {
                // Strafe for ~2 feet (0.6 meters)
                auto pose = m_drive.GetPose();
                return std::abs(pose.Y().value()) >= 0.6;
            },
            {&m_drive}
        ),

        // 8. Rotate chassis back 90 degrees counter-clockwise (original orientation)
        frc2::FunctionalCommand(
            [this] {
                m_drive.ResetOdometry(frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}});
            },
            [this] {
                // Rotate counter-clockwise (positive rotation)
                m_drive.Drive(0_mps, 0_mps, 1.5_rad_per_s, false);
            },
            [this](bool) { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); },
            [this] {
                // End when rotated back ~90 degrees
                auto heading = m_drive.GetHeading();
                return heading.value() >= 85.0; // Close to +90 degrees
            },
            {&m_drive}
        ),

        // 9. Reset odometry for return trip
        frc2::InstantCommand([this] {
            m_drive.ResetOdometry(frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}});
        }, {&m_drive}),

        // 10. Drive backward toward hub, turret tracks tag 10/26 for heading correction
        frc2::FunctionalCommand(
            [this] {},
            [this] {
                double yawCorrection = 0.0;
                int targetTag = GetHubAllianceTag2(); // Tag 10 for red, 26 for blue

                if (m_camera.detection.Get() && m_camera.tagId.Get() == targetTag) {
                    m_turret.PointAtAprilTag(m_camera.yaw.Get());
                    yawCorrection = -m_camera.yaw.Get() * 0.05; // Negative because driving backward
                }

                // Drive backward toward hub
                m_drive.Drive(-1.0_mps, 0_mps, units::radians_per_second_t{yawCorrection}, false);
            },
            [this](bool) { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); },
            [this] {
                // End when traveled ~15 feet back
                auto pose = m_drive.GetPose();
                double distance = std::sqrt(std::pow(pose.X().value(), 2) + std::pow(pose.Y().value(), 2));
                return distance >= 4.57; // 15 feet in meters
            },
            {&m_drive, &m_camera, &m_turret}
        ),

        // 11. Final turret alignment to hub tag 10/26
        frc2::FunctionalCommand(
            [this] {},
            [this] {
                int targetTag = GetHubAllianceTag2();
                if (m_camera.detection.Get() && m_camera.tagId.Get() == targetTag) {
                    m_turret.PointAtAprilTag(m_camera.yaw.Get());
                }
            },
            [this](bool) {},
            [this] {
                int targetTag = GetHubAllianceTag2();
                return m_camera.detection.Get() &&
                       m_camera.tagId.Get() == targetTag &&
                       std::abs(m_camera.yaw.Get()) < 2.0; // Within 2 degrees
            },
            {&m_camera, &m_turret}
        ),

        // 12. Shoot fuel into hub
        frc2::InstantCommand([this] { m_shooter.Shoot(); }, {&m_shooter}),
        frc2::WaitCommand(2.0_s),
        frc2::InstantCommand([this] { m_shooter.Stop(); }, {&m_shooter}),

        // 13. Stop everything
        frc2::InstantCommand([this] {
            m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false);
        }, {&m_drive})
    );
}

// Neutral Zone: Drive forward, shoot, cross bump at angle, collect fuel,
// cross second bump, shoot again
frc2::Command* RobotContainer::NeutralZoneAuto() {
    return new frc2::SequentialCommandGroup(
        // 1. Reset odometry at start (facing away from bump, shooter toward hub)
        frc2::InstantCommand([this] {
            m_drive.ResetOdometry(frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}});
        }, {&m_drive}),

        // 2. Drive forward 5-6 feet to see AprilTags
        frc2::FunctionalCommand(
            [this] {},
            [this] {
                m_drive.Drive(1.0_mps, 0_mps, 0_rad_per_s, false);
            },
            [this](bool) { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); },
            [this] {
                auto pose = m_drive.GetPose();
                double distance = std::sqrt(std::pow(pose.X().value(), 2) + std::pow(pose.Y().value(), 2));
                return distance >= 1.7; // ~5.5 feet in meters
            },
            {&m_drive}
        ),

        // 3. Align turret to hub tag 9/25 and shoot
        frc2::FunctionalCommand(
            [this] {},
            [this] {
                int targetTag = GetHubAllianceTag1(); // Tag 9 for red, 25 for blue
                if (m_camera.detection.Get() && m_camera.tagId.Get() == targetTag) {
                    m_turret.PointAtAprilTag(m_camera.yaw.Get());
                }
            },
            [this](bool) {},
            [this] {
                int targetTag = GetHubAllianceTag1();
                return m_camera.detection.Get() &&
                       m_camera.tagId.Get() == targetTag &&
                       std::abs(m_camera.yaw.Get()) < 2.0;
            },
            {&m_camera, &m_turret}
        ),
        frc2::InstantCommand([this] { m_shooter.Shoot(); }, {&m_shooter}),
        frc2::WaitCommand(2.0_s),
        frc2::InstantCommand([this] { m_shooter.Stop(); }, {&m_shooter}),

        // 4. Rotate chassis 45 degrees counter-clockwise (left) for angled bump crossing
        frc2::FunctionalCommand(
            [this] {
                m_drive.ResetOdometry(frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}});
            },
            [this] {
                m_drive.Drive(0_mps, 0_mps, 1.0_rad_per_s, false); // Rotate left
            },
            [this](bool) { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); },
            [this] {
                auto heading = m_drive.GetHeading();
                return heading.value() >= 40.0; // ~45 degrees left
            },
            {&m_drive}
        ),

        // 5. Drive backward over bump (~8 feet) while maintaining angle
        //    Travel direction is backward but chassis is angled 45 degrees
        frc2::FunctionalCommand(
            [this] {
                m_drive.ResetOdometry(frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}});
            },
            [this] {
                // Drive backward-right relative to chassis (net motion is backward in field)
                // Chassis is rotated 45 deg left, so to go "backward" we need diagonal motion
                m_drive.Drive(-0.7_mps, -0.7_mps, 0_rad_per_s, false);
            },
            [this](bool) { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); },
            [this] {
                auto pose = m_drive.GetPose();
                double distance = std::sqrt(std::pow(pose.X().value(), 2) + std::pow(pose.Y().value(), 2));
                return distance >= 2.44; // ~8 feet in meters
            },
            {&m_drive}
        ),

        // 6. Finish rotating left so front faces right (along neutral zone)
        //    Need to rotate another ~45 degrees to complete 90 degree turn
        frc2::FunctionalCommand(
            [this] {
                m_drive.ResetOdometry(frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}});
            },
            [this] {
                m_drive.Drive(0_mps, 0_mps, 1.0_rad_per_s, false); // Continue rotating left
            },
            [this](bool) { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); },
            [this] {
                auto heading = m_drive.GetHeading();
                return heading.value() >= 40.0; // Another ~45 degrees
            },
            {&m_drive}
        ),

        // 7. Drive forward ~20 feet while running intake to collect fuel
        frc2::InstantCommand([this] { m_intake.Run(); }, {&m_intake}),
        frc2::FunctionalCommand(
            [this] {
                m_drive.ResetOdometry(frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}});
            },
            [this] {
                m_drive.Drive(1.0_mps, 0_mps, 0_rad_per_s, false); // Drive forward
            },
            [this](bool) { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); },
            [this] {
                auto pose = m_drive.GetPose();
                double distance = std::sqrt(std::pow(pose.X().value(), 2) + std::pow(pose.Y().value(), 2));
                return distance >= 6.1; // ~20 feet in meters
            },
            {&m_drive}
        ),
        frc2::InstantCommand([this] { m_intake.Stop(); }, {&m_intake}),

        // 8. Rotate chassis 45 degrees left for second bump crossing
        frc2::FunctionalCommand(
            [this] {
                m_drive.ResetOdometry(frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}});
            },
            [this] {
                m_drive.Drive(0_mps, 0_mps, 1.0_rad_per_s, false); // Rotate left
            },
            [this](bool) { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); },
            [this] {
                auto heading = m_drive.GetHeading();
                return heading.value() >= 40.0; // ~45 degrees
            },
            {&m_drive}
        ),

        // 9. Drive over second bump (~8 feet) - direction is to the right (now forward)
        frc2::FunctionalCommand(
            [this] {
                m_drive.ResetOdometry(frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}});
            },
            [this] {
                // Drive forward-right relative to chassis
                m_drive.Drive(0.7_mps, -0.7_mps, 0_rad_per_s, false);
            },
            [this](bool) { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); },
            [this] {
                auto pose = m_drive.GetPose();
                double distance = std::sqrt(std::pow(pose.X().value(), 2) + std::pow(pose.Y().value(), 2));
                return distance >= 2.44; // ~8 feet in meters
            },
            {&m_drive}
        ),

        // 10. Align turret to hub tag 10/26 and shoot
        frc2::FunctionalCommand(
            [this] {},
            [this] {
                int targetTag = GetHubAllianceTag2(); // Tag 10 for red, 26 for blue
                if (m_camera.detection.Get() && m_camera.tagId.Get() == targetTag) {
                    m_turret.PointAtAprilTag(m_camera.yaw.Get());
                }
            },
            [this](bool) {},
            [this] {
                int targetTag = GetHubAllianceTag2();
                return m_camera.detection.Get() &&
                       m_camera.tagId.Get() == targetTag &&
                       std::abs(m_camera.yaw.Get()) < 2.0;
            },
            {&m_camera, &m_turret}
        ),
        frc2::InstantCommand([this] { m_shooter.Shoot(); }, {&m_shooter}),
        frc2::WaitCommand(2.0_s),
        frc2::InstantCommand([this] { m_shooter.Stop(); }, {&m_shooter}),

        // 11. Stop everything
        frc2::InstantCommand([this] {
            m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false);
        }, {&m_drive})
    );
}

// Side Climb: Start near hub, shoot, navigate to ladder, and climb
frc2::Command* RobotContainer::SideClimbAuto() {
    return new frc2::SequentialCommandGroup(
        // 1. Reset odometry (start near hub, shooter facing hub behind)
        frc2::InstantCommand([this] {
            m_drive.ResetOdometry(frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}});
        }, {&m_drive}),

        // 2. Drive forward 5 feet (away from hub)
        frc2::FunctionalCommand(
            [this] {},
            [this] {
                m_drive.Drive(1.0_mps, 0_mps, 0_rad_per_s, false);
            },
            [this](bool) { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); },
            [this] {
                auto pose = m_drive.GetPose();
                double distance = std::sqrt(std::pow(pose.X().value(), 2) + std::pow(pose.Y().value(), 2));
                return distance >= 1.52; // 5 feet in meters
            },
            {&m_drive}
        ),

        // 3. Align turret to hub tag 9/25 and shoot
        frc2::FunctionalCommand(
            [this] {},
            [this] {
                int targetTag = GetHubAllianceTag1(); // Tag 9 for red, 25 for blue
                if (m_camera.detection.Get() && m_camera.tagId.Get() == targetTag) {
                    m_turret.PointAtAprilTag(m_camera.yaw.Get());
                }
            },
            [this](bool) {},
            [this] {
                int targetTag = GetHubAllianceTag1();
                return m_camera.detection.Get() &&
                       m_camera.tagId.Get() == targetTag &&
                       std::abs(m_camera.yaw.Get()) < 2.0;
            },
            {&m_camera, &m_turret}
        ),
        frc2::InstantCommand([this] { m_shooter.Shoot(); }, {&m_shooter}),
        frc2::WaitCommand(2.0_s),
        frc2::InstantCommand([this] { m_shooter.Stop(); }, {&m_shooter}),

        // 4. Rotate chassis 180 degrees (shooter now faces ladder direction)
        frc2::FunctionalCommand(
            [this] {
                m_drive.ResetOdometry(frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}});
            },
            [this] {
                m_drive.Drive(0_mps, 0_mps, 1.5_rad_per_s, false); // Rotate left
            },
            [this](bool) { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); },
            [this] {
                auto heading = m_drive.GetHeading();
                return heading.value() >= 175.0; // ~180 degrees
            },
            {&m_drive}
        ),

        // 5. Drive toward ladder using tags 16/32, stop at 3 feet
        frc2::FunctionalCommand(
            [this] {},
            [this] {
                int tag1 = GetLadderTag1(); // Tag 15 for red, 31 for blue
                int tag2 = GetLadderTag2(); // Tag 16 for red, 32 for blue
                int tagId = m_camera.tagId.Get();
                double yawCorrection = 0.0;

                if (m_camera.detection.Get() && (tagId == tag1 || tagId == tag2)) {
                    // Use yaw to steer toward ladder
                    yawCorrection = m_camera.yaw.Get() * 0.05;
                }

                // Drive forward toward ladder (shooter is now at front after 180 rotation)
                m_drive.Drive(0.8_mps, 0_mps, units::radians_per_second_t{yawCorrection}, false);
            },
            [this](bool) { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); },
            [this] {
                int tag1 = GetLadderTag1();
                int tag2 = GetLadderTag2();
                int tagId = m_camera.tagId.Get();
                return m_camera.detection.Get() &&
                       (tagId == tag1 || tagId == tag2) &&
                       m_camera.distance.Get() <= 3.0; // Stop at 3 feet
            },
            {&m_drive, &m_camera}
        ),

        // 6. Strafe sideways to center on AprilTag (use yaw to determine direction)
        frc2::FunctionalCommand(
            [this] {},
            [this] {
                int tag1 = GetLadderTag1();
                int tag2 = GetLadderTag2();
                int tagId = m_camera.tagId.Get();

                if (m_camera.detection.Get() && (tagId == tag1 || tagId == tag2)) {
                    double yaw = m_camera.yaw.Get();
                    // Strafe based on yaw: positive yaw = tag to right = strafe right
                    if (std::abs(yaw) > 1.0) { // Only strafe if not centered
                        double strafeSpeed = (yaw > 0) ? -0.3 : 0.3; // Right is negative Y
                        m_drive.Drive(0_mps, units::meters_per_second_t{strafeSpeed}, 0_rad_per_s, false);
                    } else {
                        m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false);
                    }
                }
            },
            [this](bool) { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); },
            [this] {
                int tag1 = GetLadderTag1();
                int tag2 = GetLadderTag2();
                int tagId = m_camera.tagId.Get();
                return m_camera.detection.Get() &&
                       (tagId == tag1 || tagId == tag2) &&
                       std::abs(m_camera.yaw.Get()) < 1.0; // Centered within 1 degree
            },
            {&m_drive, &m_camera}
        ),

        // 7. Drive backward 6 inches toward ladder
        frc2::FunctionalCommand(
            [this] {
                m_drive.ResetOdometry(frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}});
            },
            [this] {
                m_drive.Drive(-0.3_mps, 0_mps, 0_rad_per_s, false); // Slow backward
            },
            [this](bool) { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); },
            [this] {
                auto pose = m_drive.GetPose();
                double distance = std::sqrt(std::pow(pose.X().value(), 2) + std::pow(pose.Y().value(), 2));
                return distance >= 0.15; // 6 inches in meters
            },
            {&m_drive}
        ),

        // 8. Activate climb
        frc2::InstantCommand([this] { m_climber.Climb(); }, {&m_climber}),
        frc2::WaitCommand(1.0_s), // Climb duration - adjust as needed
        frc2::InstantCommand([this] { m_climber.Stop(); }, {&m_climber}),

        // 9. Stop everything
        frc2::InstantCommand([this] {
            m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false);
        }, {&m_drive})
    );
}
