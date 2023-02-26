// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.drive.DefaultDriveUpdatePose;
import frc.robot.commands.drive.SetDriveTarget;
import frc.robot.commands.floorArm.FloorMoveArmToPostion;
import frc.robot.commands.intake.armIntake.DropCone;
import frc.robot.commands.intake.armIntake.DropCube;
import frc.robot.commands.intake.armIntake.IntakeCone;
import frc.robot.commands.intake.armIntake.IntakeCube;
import frc.robot.commands.intake.floor.FloorDrop;
import frc.robot.commands.intake.floor.FloorPickup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FloorArmSubsystem;
import frc.robot.subsystems.FloorIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;
import frc.robot.util.AutoGenerator;
import frc.robot.util.DPadButton;
import frc.robot.util.JoystickModification;

public class RobotContainer {
  private final Vision vision = new Vision();
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final Intake intake = new Intake();
  private final FloorIntake floorIntake = new FloorIntake();
  private final FloorArmSubsystem floorArm = new FloorArmSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final AutoGenerator autoGenerator = new AutoGenerator(drivetrainSubsystem);

  private final JoystickModification mod = new JoystickModification();

  private final Joystick driverController = new Joystick(Constants.DRIVER_CONTROLLER);
  private final Joystick operatorController = new Joystick(Constants.OPERATOR_CONTROLLER);
  
  private final JoystickButton driverControllerBackButton = new JoystickButton(driverController, Constants.BACK_BUTTON);
  private final JoystickButton driverControllerYButton = new JoystickButton(driverController, Constants.Y_BUTTON);
  private final JoystickButton driverControllerAButton = new JoystickButton(driverController, Constants.A_BUTTON);

  private final JoystickButton operatorControllerBButton = new JoystickButton(operatorController, Constants.B_BUTTON);
  private final JoystickButton operatorControllerYButton = new JoystickButton(operatorController, Constants.Y_BUTTON);
  private final JoystickButton operatorControllerAButton = new JoystickButton(operatorController, Constants.A_BUTTON);
  private final JoystickButton operatorControllerXButton = new JoystickButton(operatorController, Constants.X_BUTTON);
  private final JoystickButton operatorControllerLeftBumper = new JoystickButton(operatorController, Constants.LEFT_BUMPER);
  private final JoystickButton operatorControllerRightBumper = new JoystickButton(operatorController, Constants.RIGHT_BUMPER);
  private final JoystickButton operatorControllerLeftJoystickButton = new JoystickButton(operatorController, Constants.LEFT_JOYSTICK_BUTTON);

  private final DPadButton operatorDPadUp = new DPadButton(operatorController, DPadButton.Direction.UP);
  private final DPadButton operatorDPadRight = new DPadButton(operatorController, DPadButton.Direction.RIGHT);
  private final DPadButton operatorDPadDown = new DPadButton(operatorController, DPadButton.Direction.DOWN);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    //driver
    drivetrainSubsystem.setDefaultCommand(new DefaultDriveUpdatePose(
        vision, 
        drivetrainSubsystem,
        () -> mod.modifyAxis(driverController.getRawAxis(Constants.LEFT_X_AXIS))
        * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -mod.modifyAxis(driverController.getRawAxis(Constants.LEFT_Y_AXIS))
        * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -mod.modifyAxis(driverController.getRawAxis(Constants.RIGHT_X_AXIS))
        * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
    );
    driverControllerBackButton.onTrue(new InstantCommand(drivetrainSubsystem::zeroGyroscope));
    driverControllerAButton.onTrue(new SetDriveTarget(drivetrainSubsystem, 0));
    driverControllerYButton.onTrue(new SetDriveTarget(drivetrainSubsystem, 180));

    //operator
        //floor arm
        operatorControllerLeftJoystickButton.onTrue(new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_INSIDE_ROBOT));
        operatorControllerRightBumper.onTrue(new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_OUTSIDE_ROBOT));

        //floor intake
        operatorControllerRightBumper.whileTrue(new FloorPickup(floorIntake));
        operatorControllerLeftBumper.whileTrue(new FloorDrop(floorIntake));

        //primary arm
        operatorDPadUp.onTrue(
          new SequentialCommandGroup(
            new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_INSIDE_ROBOT),
            new MoveArmToPosition(arm, Constants.ARM_SCORE_HIGH)
        ));

        operatorDPadRight.onTrue(
          new SequentialCommandGroup(
            new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_INSIDE_ROBOT),
            new MoveArmToPosition(arm, Constants.ARM_SCORE_MID)
        ));

        operatorDPadDown.onTrue(new MoveArmToPosition(arm, Constants.ARM_INSIDE_ROBOT));
        
        //primary intake
        operatorControllerYButton.whileTrue(new DropCube(intake));
        operatorControllerAButton.whileTrue(new IntakeCube(intake));
        operatorControllerXButton.whileTrue(new DropCone(intake));
        operatorControllerBButton.whileTrue(new IntakeCone(intake));
  }

  public Command getAutonomousCommand() {
    return autoGenerator.getAutoCommand();
  }
}
