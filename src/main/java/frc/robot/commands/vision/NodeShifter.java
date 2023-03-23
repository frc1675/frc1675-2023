// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import java.util.Arrays;
import frc.robot.Constants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class NodeShifter extends CommandBase {
  /** Creates a new NodeShifter. */
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final Vision vision;
  boolean shiftRight;
  public NodeShifter(Vision vision, DrivetrainSubsystem drivetrainSubsystem, boolean shiftRight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vision = vision;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.shiftRight = shiftRight;
    addRequirements(this.drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(Arrays.asList(Constants.SCORE_APRIL_TAGS).contains(vision.getTargetID()) && shiftRight == true){
      //drivetrainSubsystem.resetPose(vision.getBotpose());
      Translation2d nodeShift = new Translation2d(drivetrainSubsystem.getPose().getX(), drivetrainSubsystem.getPose().getY() - Constants.NODE_SHIFT_DISTANCE_METERS);
      drivetrainSubsystem.setRotationTarget(Rotation2d.fromDegrees(0)); 
      drivetrainSubsystem.setTranslationTarget(nodeShift);
    }else if(Arrays.asList(Constants.SCORE_APRIL_TAGS).contains(vision.getTargetID()) && shiftRight == false){
      //drivetrainSubsystem.resetPose(vision.getBotpose());
      Translation2d nodeShift = new Translation2d(drivetrainSubsystem.getPose().getX(), drivetrainSubsystem.getPose().getY() + Constants.NODE_SHIFT_DISTANCE_METERS);
      drivetrainSubsystem.setRotationTarget(Rotation2d.fromDegrees(0));
      drivetrainSubsystem.setTranslationTarget(nodeShift);

    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
