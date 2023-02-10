// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Vision;

public class VisionPoseUpdate extends CommandBase {

  private Vision vision;
  private DrivetrainSubsystem drivetrainSubsystem;

  public VisionPoseUpdate(Vision vision, DrivetrainSubsystem drivetrainSubsystem) {
    this.vision = vision;
    this.drivetrainSubsystem = drivetrainSubsystem;
  }
  
  @Override
  public void execute() {
    if(vision.hasTarget()) {
      drivetrainSubsystem.resetPose(vision.getBotpose());
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
