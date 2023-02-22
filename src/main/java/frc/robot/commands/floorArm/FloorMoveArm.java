// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.floorArm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.FloorArmSubsystem;

public class FloorMoveArm extends CommandBase {
  /** Creates a new moveArm. */
 private FloorArmSubsystem floorArm;
 private DoubleSupplier armSpeed;

  public FloorMoveArm(FloorArmSubsystem arm,DoubleSupplier armSpeed ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.floorArm = floorArm; 
    this.armSpeed = armSpeed;
    addRequirements(this.floorArm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }
    
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double armPower = armSpeed.getAsDouble()*Constants.ARM_POWER_SCALING;
    floorArm.moveArm(armPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    floorArm.moveArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
