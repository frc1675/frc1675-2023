// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.floorArm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FloorArmSubsystem;

public class FloorMoveArmToPostion extends CommandBase {
  private FloorArmSubsystem floorArm;
  private double targetPosition;
  /** Creates a new MoveArmToPostion. */
  public FloorMoveArmToPostion(FloorArmSubsystem floorArm, double targetPosition) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.floorArm=floorArm;
    this.targetPosition=targetPosition;
    addRequirements(floorArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    floorArm.setTargetPosition(targetPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    floorArm.moveArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
