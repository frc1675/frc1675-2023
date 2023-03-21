// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake.armIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeCone extends CommandBase {
  /** Creates a new ConeUtil. */
  private final Intake intake;
  private final boolean fast;

  public IntakeCone(Intake intake) {
    this(intake, true);
  }

  public IntakeCone(Intake intake, boolean fast) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.fast = fast;
    addRequirements(this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(fast) {
      intake.conePickup(Constants.INTAKE_SPEED);
    }else {
      intake.conePickup(Constants.INTAKE_SPEED_SLOW);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
