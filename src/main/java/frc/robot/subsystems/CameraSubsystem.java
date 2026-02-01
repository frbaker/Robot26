// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {
    private final NetworkTableInstance inst;
    private final NetworkTable table;

    public final BooleanSubscriber detection;
    public final IntegerSubscriber tagId;
    public final DoubleSubscriber distance;
    public final DoubleSubscriber yaw;

    /** Creates a new CameraSubsystem. */
    public CameraSubsystem() {
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("aprilTags");
        tagId = table.getIntegerTopic("tagId").subscribe(0);
        detection = table.getBooleanTopic("detection").subscribe(false);
        distance = table.getDoubleTopic("distance").subscribe(0.0);
        yaw = table.getDoubleTopic("yaw").subscribe(0.0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void putStuffOnSmartDashboard() {
        SmartDashboard.putNumber("tagId", tagId.get());
        SmartDashboard.putBoolean("detection", detection.get());
        SmartDashboard.putNumber("distance", distance.get());
        SmartDashboard.putNumber("yaw", yaw.get());
    }

    public double getYaw() {
        return yaw.get();
    }
}
