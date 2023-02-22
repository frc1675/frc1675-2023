// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArm extends CommandBase {
  /** Creates a new moveArm. */
 private ArmSubsystem arm;
 private DoubleSupplier armSpeed;

  public MoveArm(ArmSubsystem arm,DoubleSupplier armSpeed ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm; 
    this.armSpeed = armSpeed;
    addRequirements(this.arm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }
    
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double armPower = armSpeed.getAsDouble()*Constants.ARM_POWER_SCALING;
    arm.moveArm(armPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.moveArm(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
