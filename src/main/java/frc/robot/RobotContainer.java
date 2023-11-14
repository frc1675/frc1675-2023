// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.DefaultDriveUpdatePose;
import frc.robot.commands.drive.SetDriveRotationTarget;
import frc.robot.commands.claw.ExtendClaw;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RobotContainer {
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final Joystick driverController = new Joystick(Constants.DRIVER_CONTROLLER);
  private final JoystickButton driverControllerXButton = new JoystickButton(driverController, Constants.X_BUTTON);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
        drivetrainSubsystem.setDefaultCommand(new DefaultDriveUpdatePose(
            drivetrainSubsystem,
            () -> -mod.modifyAxis(driverController.getRawAxis(Constants.LEFT_Y_AXIS))
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -mod.modifyAxis(driverController.getRawAxis(Constants.LEFT_X_AXIS))
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -mod.modifyAxis(driverController.getRawAxis(Constants.RIGHT_X_AXIS))
            * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            () -> mod.modifyAxis(driverController.getRawAxis(Constants.RIGHT_TRIGGER))
            )
          );
        driverControllerXButton.toggleOnTrue(new ExtendClaw());
  }

  public void disableDrivetrainTargets() {
    drivetrainSubsystem.setBalanceTarget(null);
    drivetrainSubsystem.setRotationTarget(null);
    drivetrainSubsystem.setTranslationTarget(null);
  }
}
