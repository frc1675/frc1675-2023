// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake.floor;
import frc.robot.subsystems.FloorIntake;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FloorSetSpeed extends CommandBase {
  private FloorIntake floorIntake;
  private double speed;

  public FloorSetSpeed(FloorIntake floorIntake, double speed) {
    this.floorIntake = floorIntake;
    this.speed = speed;
    addRequirements(this.floorIntake);
  }

  @Override
  public void initialize() {
    floorIntake.intakeDrop(speed);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
