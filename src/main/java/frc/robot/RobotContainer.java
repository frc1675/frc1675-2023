// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.FloorIntakeIn;
import frc.robot.commands.FloorIntakeOut;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ToggleRotationTarget;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FloorIntake;
import frc.robot.subsystems.Vision;
import frc.robot.util.JoystickModification;

public class RobotContainer {
  private final Vision vision = new Vision();
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final JoystickModification mod = new JoystickModification();

  /* 
   * Ground Intake Controller Operation:
   * -  Push Button once, start the intake motor and set to suck inwards
   * -  Push the button at anytime during the live state to switch directions
   * -  Long press the button to shut down the motor
  */

  private final Joystick driverController = new Joystick(Constants.DRIVER_CONTROLLER);
  private final Joystick operatorController = new Joystick(Constants.OPERATOR_CONTROLLER);
  private final JoystickButton driverControllerBackButton = new JoystickButton(driverController, Constants.BACK_BUTTON);
  private final JoystickButton driverControllerBButton = new JoystickButton(driverController, Constants.B_BUTTON);
  private final JoystickButton driverControllerYButton = new JoystickButton(driverController, Constants.Y_BUTTON);
  private final JoystickButton driverControllerAButton = new JoystickButton(driverController, Constants.A_BUTTON);
  private final JoystickButton driverControllerXButton = new JoystickButton(driverController, Constants.X_BUTTON);
  private final JoystickButton operatorControllerBButton = new JoystickButton(operatorController, Constants.B_BUTTON);
  private final JoystickButton operatorControllerYButton = new JoystickButton(operatorController, Constants.Y_BUTTON);
  private final JoystickButton operatorControllerAButton = new JoystickButton(operatorController, Constants.A_BUTTON);
  private final JoystickButton operatorControllerXButton = new JoystickButton(operatorController, Constants.X_BUTTON);
  private final FloorIntake floorIntake = new FloorIntake();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        drivetrainSubsystem,
        () -> -mod.modifyAxis(driverController.getRawAxis(Constants.LEFT_Y_AXIS))
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -mod.modifyAxis(driverController.getRawAxis(Constants.LEFT_X_AXIS))
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -mod.modifyAxis(driverController.getRawAxis(Constants.RIGHT_X_AXIS))
            * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    driverControllerBackButton.onTrue(new InstantCommand(drivetrainSubsystem::zeroGyroscope));
  
    driverControllerYButton.onTrue(new ToggleRotationTarget(drivetrainSubsystem, () -> vision.getTargetXOffsetDegrees()));
    driverControllerBButton.onTrue(new ToggleRotationTarget(drivetrainSubsystem, () -> Constants.DRIVE_ROTATION_TARGET_DEGREES));
    operatorControllerXButton.onTrue(new FloorIntakeIn(floorIntake));
    operatorControllerAButton.onTrue(new FloorIntakeOut(floorIntake));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
