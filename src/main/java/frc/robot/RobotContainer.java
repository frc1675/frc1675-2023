// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.claw.ToggleClaw;
import frc.robot.subsystems.ClawIntake;

public class RobotContainer {
  private final Joystick driverController = new Joystick(Constants.DRIVER_CONTROLLER);
  private final JoystickButton driverControllerXButton = new JoystickButton(driverController, Constants.X_BUTTON);
  private final ClawIntake intake = new ClawIntake(1, 0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
        driverControllerXButton.toggleOnTrue(new ToggleClaw(intake));
  }
}
