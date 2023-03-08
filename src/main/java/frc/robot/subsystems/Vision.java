// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private NetworkTable table;
  private NetworkTableEntry targetValid;
  private NetworkTableEntry targetXOffset;
  private NetworkTableEntry targetArea; // proxy for distance from target
  private NetworkTableEntry targetID;
  private NetworkTableEntry botpose;
  

  public static enum CamMode {
    VISION,
    DRIVER
  }
  public static enum LEDMode {
    PIPELINE(0),
    OFF(1),
    BLINK(2),
    ON(3);

    int value;

    private LEDMode(int value){
      this.value = value;
    }
  }

  public Vision() {
    table = NetworkTableInstance.getDefault().getTable("limelight-ups");
    targetValid = table.getEntry("tv");
    targetID  = table.getEntry("tid");
    targetXOffset = table.getEntry("tx");
    targetArea = table.getEntry("ta");
    if(DriverStation.getAlliance() == Alliance.Red) {
      botpose = table.getEntry("botpose_wpired");
    }else {
      botpose = table.getEntry("botpose_wpiblue");
    }
  }

  public Pose2d getBotpose() {
    double[] arr = botpose.getDoubleArray(new double[6]);
    return new Pose2d(arr[0], arr[1], Rotation2d.fromDegrees(arr[5]));
  }

  public int getTargetID() {
    return (int) targetID.getInteger(0);
  }

  public double getTargetArea() {
    return targetArea.getDouble(0);
  }

  public double getTargetXOffsetDegrees() {
    return targetXOffset.getDouble(0);
  }

  public boolean hasTarget() {
    return targetValid.getDouble(0) == 1;
  }

  public void setMode(CamMode mode) {
    if(mode == CamMode.VISION) {
      table.getEntry("camMode").setNumber(0);
    }else {
      table.getEntry("camMode").setNumber(1);
    }
  }
  
  
  public void setLEDMode(LEDMode ledmode) {
    if(ledmode == LEDMode.PIPELINE){
      table.getEntry("LEDMode").setNumber(0);
    }
    else if(ledmode == LEDMode.ON){

      table.getEntry("LEDMode").setNumber(1); 
    }
    else if(ledmode == LEDMode.BLINK){
      table.getEntry("LEDMode").setNumber(2); 
    }
    else{
      table.getEntry("LEDMode").setNumber(3);
    }

  }
}
