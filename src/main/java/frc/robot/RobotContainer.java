// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.RotateCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.util.JoystickModification;

public class RobotContainer {
  private final Vision vision = new Vision();
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();

  private final Joystick driverController = new Joystick(Constants.DRIVER_CONTROLLER);
  private final JoystickButton aButton = new JoystickButton(driverController, Constants.A_BUTTON);
  private final JoystickButton backButton = new JoystickButton(driverController, Constants.BACK_BUTTON);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        drivetrainSubsystem,
        () -> -JoystickModification.modifyAxis(driverController.getRawAxis(Constants.LEFT_Y_AXIS))
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -JoystickModification.modifyAxis(driverController.getRawAxis(Constants.LEFT_X_AXIS))
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -JoystickModification.modifyAxis(driverController.getRawAxis(Constants.RIGHT_X_AXIS))
            * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    backButton.onTrue(new InstantCommand(drivetrainSubsystem::zeroGyroscope));
    aButton.whileTrue(new RotateCommand(drivetrainSubsystem, new Rotation2d(0)));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
