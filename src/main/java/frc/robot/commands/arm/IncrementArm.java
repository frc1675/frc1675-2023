// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class IncrementArm extends CommandBase {

 private ArmSubsystem arm;
 private DoubleSupplier power;
 private double startPos;

  public IncrementArm(ArmSubsystem arm, DoubleSupplier power) {
    this.arm = arm; 
    this.power = power;
    startPos = Constants.ARM_HUMAN_PLAYER_POSITION;
    addRequirements(this.arm);

  }

  @Override
  public void execute() {
    arm.setTargetPosition(startPos + Constants.ARM_ENCODER_COUNT_MAX_DIFF * power.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    arm.setTargetPosition(startPos);
  }
}
