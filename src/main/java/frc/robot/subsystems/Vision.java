// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private NetworkTable table;
  private NetworkTableEntry validTarget;
  private NetworkTableEntry xOffset;
  private NetworkTableEntry targetArea; // proxy for distance from target
  private NetworkTableEntry targetID;
  private NetworkTableEntry botpose;

  public Vision() {
    table = NetworkTableInstance.getDefault().getTable("limelight-ups");
    validTarget = table.getEntry("tv");
    targetID  = table.getEntry("tid");
    xOffset = table.getEntry("tx");
    targetArea = table.getEntry("ta");
    botpose = table.getEntry("botpose");
  }

  public Pose2d getBotpose() {
    double[] arr = botpose.getDoubleArray(new double[6]);
    return new Pose2d(arr[0], arr[1], new Rotation2d(arr[3], arr[4]));
  }

  public int getTargetID() {
    return (int) targetID.getInteger(0);
  }

  public double getTargetArea() {
    return targetArea.getDouble(0);
  }

  public Rotation2d getXOffset() {
    return Rotation2d.fromDegrees(xOffset.getDouble(0));
  }

  public boolean hasTarget() {
    return validTarget.getDouble(0) == 1;
  }

  public void setMode(CamMode mode) {
    if(mode == CamMode.VISION) {
      table.getEntry("camMode").setNumber(0);
    }else {
      table.getEntry("camMode").setNumber(1);
    }
  }

  public static enum CamMode {
    VISION,
    DRIVER
  }
  @Override
  public void periodic() {
   
  }

}
