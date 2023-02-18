// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmPosition extends CommandBase {
  /** Creates a new ArmPosition. */
  private ArmSubsystem arm;
  private DoubleSupplier armValue;
  
  
  

  public ArmPosition(ArmSubsystem arm, double armPosition, boolean canBeFinished) {


    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

    
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    double armPower = armValue.getAsDouble()*Constants.ARM_POWER_SCALING;
    arm.moveArm(armPower);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
