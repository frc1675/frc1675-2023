// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;
import frc.robot.Constants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.Arrays;
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
    addRequirements(this.vision, this.drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if(Arrays.asList(Constants.SCORE_APRIL_TAGS).contains(vision.getTargetID()) 
    // && drivetrainSubsystem.getPose().getX() == Constants.){

    // }
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
