// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the Robot
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_drive = new DriveSubsystem();
    private final CameraSubsystem m_camera = new CameraSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    private final TurretSubsystem m_turret = new TurretSubsystem();

    // The driver's controller
    private final XboxController m_driverController =
        new XboxController(OIConstants.kDriverControllerPort);
    private final XboxController m_coDriverController =
        new XboxController(OIConstants.kCoDriverControllerPort);

    private boolean fieldRelative = false;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        m_drive.setDefaultCommand(
            new RunCommand(
                () -> m_drive.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    fieldRelative),
                m_drive));

        m_camera.setDefaultCommand(
            new RunCommand(() -> m_camera.putStuffOnSmartDashboard(), m_camera));
    }

    /**
     * Use this method to define your button->command mappings.
     */
    private void configureButtonBindings() {
        // Set X formation when right bumper is held
        new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
            .whileTrue(new RunCommand(() -> m_drive.setX(), m_drive));

        // Shoot when A button is pressed, stop when released
        new JoystickButton(m_driverController, XboxController.Button.kA.value)
            .onTrue(new InstantCommand(() -> m_shooter.shoot(), m_shooter))
            .onFalse(new InstantCommand(() -> m_shooter.stop(), m_shooter));

        // Toggle field relative mode with Start button
        new JoystickButton(m_driverController, XboxController.Button.kStart.value)
            .onTrue(new InstantCommand(() -> fieldRelative = !fieldRelative));

        // Point turret at AprilTag when left bumper is held
        new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
            .whileTrue(new RunCommand(
                () -> m_turret.pointAtAprilTag(m_camera.getYaw()),
                m_turret, m_camera));
    }

    /**
     * Use this to pass the autonomous command to the main Robot class.
     */
    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(m_drive.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                config);

        var thetaController =
            new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                m_drive::getPose,
                m_drive.kDriveKinematics,
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                m_drive::setModuleStates,
                m_drive);

        // Reset odometry to the starting pose of the trajectory.
        m_drive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return new SequentialCommandGroup(
            swerveControllerCommand,
            new InstantCommand(() -> m_drive.drive(0, 0, 0, false)));
    }
}
