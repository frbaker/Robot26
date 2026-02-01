// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
    private final SparkMax m_turretMotor;
    private final PIDController anglePIDController;

    /** Creates a new TurretSubsystem. */
    public TurretSubsystem() {
        m_turretMotor = new SparkMax(TurretConstants.kTurretCanId, MotorType.kBrushless);
        anglePIDController = new PIDController(0.05, 0, 0.005);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void pointAtAprilTag(double yaw) {
        double rotation = anglePIDController.calculate(yaw, 0.0);
        m_turretMotor.set(rotation / 75);
    }
}
