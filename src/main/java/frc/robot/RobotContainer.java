// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDriveUpdatePose;
import frc.robot.commands.ToggleRotationTarget;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.util.AutoGenerator;
import frc.robot.util.JoystickModification;
import frc.robot.util.AutoGenerator.StartLocation;
import frc.robot.commands.IntakeCone;
import frc.robot.commands.IntakeCube;
import frc.robot.commands.DropCube;
import frc.robot.commands.DropCone;
import frc.robot.subsystems.Intake;

public class RobotContainer {
  private final Vision vision = new Vision();
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final JoystickModification mod = new JoystickModification();
  private final Joystick operatorController = new Joystick(Constants.OPERATOR_CONTROLLER);
  private final Joystick driverController = new Joystick(Constants.DRIVER_CONTROLLER);
  private final JoystickButton operatorControllerAButton = new JoystickButton(operatorController, Constants.A_BUTTON);
  private final JoystickButton operatorControllerBButton = new JoystickButton(operatorController, Constants.B_BUTTON);
  private final JoystickButton operatorControllerXButton = new JoystickButton(operatorController, Constants.X_BUTTON);
  private final JoystickButton operatorControllerYButton = new JoystickButton(operatorController, Constants.Y_BUTTON);
  private final Intake intake = new Intake();

  private final JoystickButton driverControllerBackButton = new JoystickButton(driverController, Constants.BACK_BUTTON);
  private final JoystickButton driverControllerBButton = new JoystickButton(driverController, Constants.B_BUTTON);
  private final JoystickButton driverControllerYButton = new JoystickButton(driverController, Constants.Y_BUTTON);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    drivetrainSubsystem.setDefaultCommand(new DefaultDriveUpdatePose(
        vision, 
        drivetrainSubsystem,
        () -> -mod.modifyAxis(driverController.getRawAxis(Constants.LEFT_Y_AXIS))
        * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -mod.modifyAxis(driverController.getRawAxis(Constants.LEFT_X_AXIS))
        * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -mod.modifyAxis(driverController.getRawAxis(Constants.RIGHT_X_AXIS))
        * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
    );

    driverControllerBackButton.onTrue(new InstantCommand(drivetrainSubsystem::zeroGyroscope));
    driverControllerYButton.onTrue(new ToggleRotationTarget(drivetrainSubsystem, () -> vision.getTargetXOffsetDegrees()));
    driverControllerBButton.onTrue(new ToggleRotationTarget(drivetrainSubsystem, () -> Constants.DRIVE_ROTATION_TARGET_DEGREES));

    operatorControllerAButton.onTrue(new DropCone(intake));
    operatorControllerBButton.onTrue(new DropCube(intake));
    operatorControllerXButton.onTrue(new IntakeCone(intake));
    operatorControllerYButton.onTrue(new IntakeCube(intake));
  }

  public Command getAutonomousCommand() {
    if(Robot.isSimulation()) {
      //drivetrainSubsystem.resetPose(new Pose2d(new Translation2d(4, -7), Rotation2d.fromRadians(0.0)));
      return new AutoGenerator(drivetrainSubsystem).getExitAndBalance(StartLocation.ONE);
    }
    return new AutoGenerator(drivetrainSubsystem).getAutoCommand();
  }
}
