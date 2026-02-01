// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax m_leftShooter;
    private final RelativeEncoder m_leftEncoder;
    private final SparkClosedLoopController m_leftController;

    private final SparkMax m_rightShooter;
    private final RelativeEncoder m_rightEncoder;
    private final SparkClosedLoopController m_rightController;

    private final SparkMax m_feederMotor;
    private final RelativeEncoder m_feederEncoder;
    private final SparkClosedLoopController m_feederController;

    /** Creates a new ShooterSubsystem. */
    public ShooterSubsystem() {
        m_leftShooter = new SparkMax(ShooterConstants.kShooterLeftCanId, MotorType.kBrushless);
        m_leftEncoder = m_leftShooter.getEncoder();
        m_leftController = m_leftShooter.getClosedLoopController();

        m_rightShooter = new SparkMax(ShooterConstants.kShooterRightCanId, MotorType.kBrushless);
        m_rightEncoder = m_rightShooter.getEncoder();
        m_rightController = m_rightShooter.getClosedLoopController();

        m_feederMotor = new SparkMax(ShooterConstants.kFeederCanId, MotorType.kBrushless);
        m_feederEncoder = m_feederMotor.getEncoder();
        m_feederController = m_feederMotor.getClosedLoopController();

        // Configure motors
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.closedLoop.pid(0.0005, 0.0000005, 0.0001);
        m_leftShooter.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.closedLoop.pid(0.0005, 0.0000005, 0.0001);
        m_rightShooter.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig.closedLoop.pid(0.0005, 0.0000005, 0.0001);
        m_feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void shoot() {
        m_leftController.setReference(-1600, ControlType.kVelocity);
        m_rightController.setReference(-1600, ControlType.kVelocity);
        m_feederController.setReference(-1200, ControlType.kVelocity);
    }

    public void stop() {
        m_leftShooter.set(0);
        m_rightShooter.set(0);
        m_feederMotor.set(0);
    }
}
