// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.FloorIntake;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToggleFloorIntake extends CommandBase {
  /** Creates a new ToggleFloorIntake. */
  private final FloorIntake floorIntake;
  public ToggleFloorIntake(FloorIntake floorIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.floorIntake = floorIntake;
    addRequirements(this.floorIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    floorIntake.intakePowerCycle();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
