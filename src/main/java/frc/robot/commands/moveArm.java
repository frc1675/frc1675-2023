// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class moveArm extends CommandBase {
  /** Creates a new moveArm. */
 private ArmSubsystem arm;
 private DoubleSupplier armValue;

  public moveArm(ArmSubsystem arm,DoubleSupplier armValue ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm; 
    this.armValue = armValue;
    addRequirements(this.arm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.unlock();
  }
    
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double armPower = armValue.getAsDouble()*Constants.ARM_POWER;
    arm.moveArm(armPower);
    // Change the constant 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.lock();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
