// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PlayerStationAutoAlign extends CommandBase {
  /** Creates a new PlayerStationAutoAlign. */
  private final Vision vision;
  private final DrivetrainSubsystem drivetrainSubsystem;
  public PlayerStationAutoAlign(DrivetrainSubsystem drivetrainSubsystem, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vision = vision;
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(this.drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(vision.getTargetID() == 4){ // Override if Blue Alliance Human Player April Tag is seen
      drivetrainSubsystem.resetPose(vision.getBotpose());
      drivetrainSubsystem.setRotationTarget(Constants.BLUE_ALLIANCE_HUMAN_BOUNDARY.getRotation());
    }else if(vision.getTargetID() == 5){ // Override if RED Alliance Human Player April Tag is seen
      drivetrainSubsystem.resetPose(vision.getBotpose());
      drivetrainSubsystem.setRotationTarget(Constants.RED_ALLIANCE_HUMAN_BOUNDARY.getRotation());
    }else if(DriverStation.getAlliance() == Alliance.Red){ // For Red Alliance
      if(drivetrainSubsystem.getPose().getX() < Constants.RED_ALLIANCE_HUMAN_BOUNDARY.getX() && drivetrainSubsystem.getPose().getY() > Constants.RED_ALLIANCE_HUMAN_BOUNDARY.getY()){
        drivetrainSubsystem.setRotationTarget(Constants.RED_ALLIANCE_HUMAN_BOUNDARY.getRotation());
      }
    }else{ // For Blue Alliance
      if(drivetrainSubsystem.getPose().getX() > Constants.BLUE_ALLIANCE_HUMAN_BOUNDARY.getX() && drivetrainSubsystem.getPose().getY() > Constants.BLUE_ALLIANCE_HUMAN_BOUNDARY.getX()){
        drivetrainSubsystem.setRotationTarget(Constants.BLUE_ALLIANCE_HUMAN_BOUNDARY.getRotation());
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
