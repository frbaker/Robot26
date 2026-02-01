// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants.ModuleConstants;

public class MAXSwerveModule {
    private final SparkMax m_drivingSpark;
    private final SparkMax m_turningSpark;

    private final RelativeEncoder m_drivingEncoder;
    private final AbsoluteEncoder m_turningAbsoluteEncoder;

    private final SparkClosedLoopController m_drivingClosedLoopController;
    private final SparkClosedLoopController m_turningClosedLoopController;

    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller.
     */
    public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
        m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
        m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

        m_drivingEncoder = m_drivingSpark.getEncoder();
        m_turningAbsoluteEncoder = m_turningSpark.getAbsoluteEncoder();

        m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
        m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

        // Configure driving motor
        SparkMaxConfig drivingConfig = new SparkMaxConfig();
        drivingConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
        drivingConfig.encoder
            .positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
            .velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
        drivingConfig.closedLoop
            .pid(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD)
            .velocityFF(ModuleConstants.kDrivingFF)
            .outputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);

        m_drivingSpark.configure(drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure turning motor
        SparkMaxConfig turningConfig = new SparkMaxConfig();
        turningConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
        turningConfig.absoluteEncoder
            .inverted(true)
            .positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
            .velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
        turningConfig.closedLoop
            .pid(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD)
            .outputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(
                ModuleConstants.kTurningEncoderPositionPIDMinInput,
                ModuleConstants.kTurningEncoderPositionPIDMaxInput);

        m_turningSpark.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_chassisAngularOffset = chassisAngularOffset;
        m_desiredState.angle = new Rotation2d(m_turningAbsoluteEncoder.getPosition());
        m_drivingEncoder.setPosition(0);
    }

    /**
     * Returns the current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            m_drivingEncoder.getVelocity(),
            new Rotation2d(m_turningAbsoluteEncoder.getPosition() - m_chassisAngularOffset));
    }

    /**
     * Returns the current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            m_drivingEncoder.getPosition(),
            new Rotation2d(m_turningAbsoluteEncoder.getPosition() - m_chassisAngularOffset));
    }

    /**
     * Sets the desired state for the module.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle =
            desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        correctedDesiredState.optimize(new Rotation2d(m_turningAbsoluteEncoder.getPosition()));

        m_drivingClosedLoopController.setReference(
            correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        m_turningClosedLoopController.setReference(
            correctedDesiredState.angle.getRadians(), ControlType.kPosition);

        m_desiredState = desiredState;
    }

    /**
     * Zeroes all the SwerveModule encoders.
     */
    public void resetEncoders() {
        m_drivingEncoder.setPosition(0);
    }
}
