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
  private final JoystickButton driverControllerBButton = new JoystickButton(driverController, Constants.B_BUTTON);
  private final JoystickButton driverControllerYButton = new JoystickButton(driverController, Constants.Y_BUTTON);
  private final JoystickButton driverControllerAButton = new JoystickButton(driverController, Constants.A_BUTTON);
  private final JoystickButton driverControllerXButton = new JoystickButton(driverController, Constants.X_BUTTON);
  private final JoystickButton driverControllerLeftBumper = new JoystickButton(driverController, Constants.LEFT_BUMPER);
  private final JoystickButton driverControllerRightBumper = new JoystickButton(driverController, Constants.RIGHT_BUMPER);

  private final DPadButton driverDPadUp = new DPadButton(operatorController, DPadButton.Direction.UP);
  private final DPadButton driverDPadLeft = new DPadButton(operatorController, DPadButton.Direction.LEFT);
  private final DPadButton driverDPadDown = new DPadButton(operatorController, DPadButton.Direction.DOWN);

  private final JoystickButton operatorControllerBButton = new JoystickButton(operatorController, Constants.B_BUTTON);
  private final JoystickButton operatorControllerYButton = new JoystickButton(operatorController, Constants.Y_BUTTON);
  private final JoystickButton operatorControllerAButton = new JoystickButton(operatorController, Constants.A_BUTTON);
  private final JoystickButton operatorControllerXButton = new JoystickButton(operatorController, Constants.X_BUTTON);

  private final DPadButton operatorDPadUp = new DPadButton(operatorController, DPadButton.Direction.UP);
  private final DPadButton operatorDPadRight = new DPadButton(operatorController, DPadButton.Direction.RIGHT);
  private final DPadButton operatorDPadLeft = new DPadButton(operatorController, DPadButton.Direction.LEFT);
  private final DPadButton operatorDPadDown = new DPadButton(operatorController, DPadButton.Direction.DOWN);

  public RobotContainer() {
    configureBindings();
  }

  private boolean armIsInsideRobot() {
    return arm.getTargetPosition() == Constants.ARM_INSIDE_ROBOT;
  }

  private boolean floorArmIsInsideRobot() {
    return floorArm.getTargetPosition() == Constants.FLOOR_ARM_INSIDE_ROBOT;
  }

  private void configureBindings() {
    //driver
    {
        //drivetrain
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
        driverControllerLeftBumper.onTrue(new SetDriveTarget(drivetrainSubsystem, 0));
        driverControllerRightBumper.onTrue(new SetDriveTarget(drivetrainSubsystem, 180));

        //floor intake
        driverDPadUp.whileTrue(
          new ConditionalCommand(
            new PrintCommand("Floor arm inside robot"),
            new FloorPickup(floorIntake), 
            () -> floorArmIsInsideRobot()
        ));
        driverDPadDown.whileTrue(
          new ConditionalCommand(
            new PrintCommand("Floor arm inside robot"),
            new FloorDrop(floorIntake), 
            () -> floorArmIsInsideRobot()
        ));
        driverDPadLeft.whileTrue(
          new ConditionalCommand(
            new PrintCommand("Floor arm inside robot"),
            new FloorDrop(floorIntake, false  ), 
            () -> floorArmIsInsideRobot()
        ));

        //main intake
        driverControllerYButton.whileTrue(
          new ConditionalCommand(
            new PrintCommand("Arm inside robot"),
            new DropCube(intake), 
            () -> armIsInsideRobot()
        ));driverControllerAButton.whileTrue(
          new ConditionalCommand(
            new PrintCommand("Arm inside robot"),
            new IntakeCube(intake), 
            () -> armIsInsideRobot()
        ));    
        driverControllerXButton.whileTrue(
          new ConditionalCommand(
            new PrintCommand("Arm inside robot"),
            new DropCone(intake), 
            () -> armIsInsideRobot()
        ));
        driverControllerBButton.whileTrue(
          new ConditionalCommand(
            new PrintCommand("Arm inside robot"),
            new IntakeCone(intake), 
            () -> armIsInsideRobot()
        ));
    }

    //operator
    {
        //floor arm
        operatorControllerYButton.onTrue(
          new SequentialCommandGroup(
            new MoveArmToPosition(arm, Constants.ARM_INSIDE_ROBOT),
            new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_GROUND_INTAKE)
          )
        );
        operatorControllerXButton.onTrue(
          new SequentialCommandGroup(
            new MoveArmToPosition(arm, Constants.ARM_INSIDE_ROBOT),
            new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_HUMAN_PLAYER)
          )
        );
        operatorControllerBButton.onTrue(
          new SequentialCommandGroup(
            new MoveArmToPosition(arm, Constants.ARM_INSIDE_ROBOT),
            new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_MIDDLE)
          )
        );
        operatorControllerAButton.onTrue(new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_INSIDE_ROBOT));

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
        operatorDPadLeft.onTrue(
          new SequentialCommandGroup(
            new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_INSIDE_ROBOT),
            new MoveArmToPosition(arm, Constants.ARM_HUMAN_PLAYER_INTAKE)
          )
        );
        operatorDPadDown.onTrue(new MoveArmToPosition(arm, Constants.ARM_INSIDE_ROBOT));
    }
  }

  public Command getAutonomousCommand() {
    return autoGenerator.getAutoCommand();
  }
}
