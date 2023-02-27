// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake.floor;
import frc.robot.Constants;
import frc.robot.subsystems.FloorIntake;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FloorDrop extends CommandBase {
  /** Creates a new FloorIntakeOut. */
  private FloorIntake floorIntake;
  private boolean fast = true;
  public FloorDrop(FloorIntake floorIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.floorIntake = floorIntake;
    addRequirements(this.floorIntake);
  }

  public FloorDrop(FloorIntake floorIntake, boolean fast) {
    this(floorIntake);
    this.fast = fast;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(fast) {
      floorIntake.intakeDrop(Constants.FLOOR_INTAKE_SPEED);
    }else {
      floorIntake.intakeDrop(Constants.FLOOR_INTAKE_SPEED);
    }
    
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
