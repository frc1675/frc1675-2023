// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake.armIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class ArmDrop extends CommandBase {
  private final Intake intake;
  private final double speed;

  public ArmDrop(Intake intake) {
    this.intake = intake;
    this.speed = Constants.INTAKE_SPEED;
  }

  public ArmDrop(Intake intake, double speed) {
    this.intake = intake;
    this.speed = Math.abs(speed);
  }

  @Override
  public void execute() {
    intake.setIntakeSpeed(-speed);
  }

  @Override
  public void end(boolean interrupted) {
    intake.intakeStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
