// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake.floor;
import frc.robot.subsystems.FloorIntake;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FloorDrop extends CommandBase {
  /** Creates a new FloorIntakeOut. */
  private FloorIntake floorIntake;
  private double speed;

  public FloorDrop(FloorIntake floorIntake, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.floorIntake = floorIntake;
    this.speed = speed;
    addRequirements(this.floorIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    floorIntake.intakeDrop(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    floorIntake.intakeStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
