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
import frc.robot.commands.arm.IncrementArm;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.drive.DefaultDriveUpdatePose;
import frc.robot.commands.drive.SetDriveRotationTarget;
import frc.robot.commands.floorArm.FloorMoveArmToPostion;
import frc.robot.commands.intake.armIntake.DropCone;
import frc.robot.commands.intake.armIntake.DropCube;
import frc.robot.commands.intake.armIntake.IntakeCone;
import frc.robot.commands.intake.armIntake.IntakeCube;
import frc.robot.commands.intake.floor.FloorDrop;
import frc.robot.commands.intake.floor.FloorPickup;
import frc.robot.commands.vision.NodeShifter;
import frc.robot.commands.vision.ToggleLED;
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
  private final AutoGenerator autoGenerator = new AutoGenerator(drivetrainSubsystem, floorArm, arm, intake, floorIntake);

  private final JoystickModification mod = new JoystickModification();

  private final Joystick driverController = new Joystick(Constants.DRIVER_CONTROLLER);
  private final Joystick operatorController = new Joystick(Constants.OPERATOR_CONTROLLER);
  
  private final JoystickButton driverControllerBackButton = new JoystickButton(driverController, Constants.BACK_BUTTON);
  private final JoystickButton driverControllerXButton = new JoystickButton(driverController, Constants.X_BUTTON);
  private final JoystickButton driverControllerAButton = new JoystickButton(driverController, Constants.A_BUTTON);
  private final JoystickButton driverControllerBButton = new JoystickButton(driverController, Constants.B_BUTTON);
  private final JoystickButton driverControllerLeftBumper = new JoystickButton(driverController, Constants.LEFT_BUMPER);
  private final JoystickButton driverControllerRightBumper = new JoystickButton(driverController, Constants.RIGHT_BUMPER);
  private final JoystickButton driverControllerStartButton = new JoystickButton(driverController, Constants.START_BUTTON);

  private final JoystickButton operatorControllerBButton = new JoystickButton(operatorController, Constants.B_BUTTON);
  private final JoystickButton operatorControllerYButton = new JoystickButton(operatorController, Constants.Y_BUTTON);
  private final JoystickButton operatorControllerAButton = new JoystickButton(operatorController, Constants.A_BUTTON);
  private final JoystickButton operatorControllerLeftBumper = new JoystickButton(operatorController, Constants.LEFT_BUMPER);
  private final JoystickButton operatorControllerRightBumper = new JoystickButton(operatorController, Constants.RIGHT_BUMPER);
  private final JoystickButton operatorControllerStartButton = new JoystickButton(operatorController, Constants.START_BUTTON);

  private final DPadButton operatorDPadUp = new DPadButton(operatorController, DPadButton.Direction.UP);
  private final DPadButton operatorDPadRight = new DPadButton(operatorController, DPadButton.Direction.RIGHT);
  private final DPadButton operatorDPadLeft = new DPadButton(operatorController, DPadButton.Direction.LEFT);
  private final DPadButton operatorDPadDown = new DPadButton(operatorController, DPadButton.Direction.DOWN);

  private final DPadButton driverDPadUp = new DPadButton(operatorController, DPadButton.Direction.UP);
  private final DPadButton driverDPadRight = new DPadButton(operatorController, DPadButton.Direction.RIGHT);
  private final DPadButton driverDPadLeft = new DPadButton(operatorController, DPadButton.Direction.LEFT);
  private final DPadButton driverDPadDown = new DPadButton(operatorController, DPadButton.Direction.DOWN);


  public RobotContainer() {
    configureBindings();
  }

  private boolean armIsExtended() {
    return arm.getTargetPosition() != Constants.ARM_INSIDE_ROBOT_POSITION;
  }

  private boolean floorArmIsExtended() {
    return floorArm.getTargetPosition() != Constants.FLOOR_ARM_INSIDE_ROBOT_POSITION;
  }

  private void configureBindings() {
    //driver
    {
        driverControllerStartButton.onTrue(new InstantCommand(drivetrainSubsystem::setBalanceTargetDefault));
        //drivetrain
        drivetrainSubsystem.setDefaultCommand(new DefaultDriveUpdatePose(
            vision, 
            drivetrainSubsystem,
            () -> -mod.modifyAxis(driverController.getRawAxis(Constants.LEFT_Y_AXIS))
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -mod.modifyAxis(driverController.getRawAxis(Constants.LEFT_X_AXIS))
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -mod.modifyAxis(driverController.getRawAxis(Constants.RIGHT_X_AXIS))
            * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            () -> mod.modifyAxis(driverController.getRawAxis(Constants.RIGHT_TRIGGER)),
            () -> armIsExtended())
        );
        driverControllerBackButton.onTrue(new InstantCommand(drivetrainSubsystem::zeroRotation));
        driverControllerLeftBumper.onTrue(new SetDriveRotationTarget(drivetrainSubsystem, 0));
        driverControllerRightBumper.onTrue(new SetDriveRotationTarget(drivetrainSubsystem, 180));

        //main intake
        driverControllerAButton.whileTrue(
          new ConditionalCommand(
            new DropCube(intake),
            new ConditionalCommand(
              new FloorDrop(floorIntake, Constants.FLOOR_INTAKE_NORMAL_SPEED),
              new PrintCommand("Cannot output"),
              ()-> floorArmIsExtended()),
            ()-> armIsExtended()
          )
        );

        driverControllerXButton.whileTrue(
          new ConditionalCommand(
            new DropCone(intake),
            new PrintCommand("Arm inside robot"), 
            () -> armIsExtended()
        ));

        driverControllerBButton.whileTrue(
          new ConditionalCommand(
            new FloorDrop(floorIntake, Constants.FLOOR_INTAKE_FAST_SPEED), 
            new PrintCommand("Floor arm inside robot"),
            () -> floorArmIsExtended()
        ));

        
    }

    //operator
    {
        //floor arm
        operatorControllerYButton.onTrue(
          new SequentialCommandGroup(
            new MoveArmToPosition(arm, Constants.ARM_INSIDE_ROBOT_POSITION),
            new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_GROUND_POSITION)
          )
        );
        operatorControllerBButton.onTrue(
          new SequentialCommandGroup(
            new MoveArmToPosition(arm, Constants.ARM_INSIDE_ROBOT_POSITION),
            new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_SHOOTING_POSITION)
          )
        );
        operatorControllerAButton.onTrue(new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_INSIDE_ROBOT_POSITION));

        //primary arm
        operatorDPadUp.onTrue(
          new SequentialCommandGroup(
            new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_INSIDE_ROBOT_POSITION),
            new MoveArmToPosition(arm, Constants.ARM_SCORE_HIGH_POSITION)
        ));
        operatorDPadRight.onTrue(
          new SequentialCommandGroup(
            new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_INSIDE_ROBOT_POSITION),
            new MoveArmToPosition(arm, Constants.ARM_SCORE_MID_POSITION)
        ));
        operatorDPadLeft.onTrue(
          new SequentialCommandGroup(
            new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_INSIDE_ROBOT_POSITION),
            new MoveArmToPosition(arm, Constants.ARM_HUMAN_PLAYER_POSITION),
            new IncrementArm(arm, () -> mod.modifyAxis(operatorController.getRawAxis(Constants.RIGHT_TRIGGER)))
          )
        );

        operatorDPadDown.onTrue(new MoveArmToPosition(arm, Constants.ARM_INSIDE_ROBOT_POSITION));



        //floor intake
        operatorControllerLeftBumper.whileTrue(
          new ConditionalCommand(
            new IntakeCone(intake),
            new PrintCommand("Arm inside robot"),
            () -> armIsExtended()
        ));
        operatorControllerRightBumper.whileTrue(
          new ConditionalCommand(
            new IntakeCube(intake),
            new ConditionalCommand(
              new FloorPickup(floorIntake),
              new PrintCommand("Cannot intake"),
              ()-> floorArmIsExtended()),
            ()-> armIsExtended()
          )
        );
        //vision
        operatorControllerStartButton.toggleOnTrue(new ToggleLED(vision));
        driverDPadLeft.onTrue(new NodeShifter(vision, drivetrainSubsystem, false));
        driverDPadRight.onTrue(new NodeShifter(vision, drivetrainSubsystem, true));
        
    }
  }

  public Command getAutonomousCommand() {
    return autoGenerator.getAutoCommand();
  }

  public void updateAutoSelectorPose() {
    autoGenerator.updateSelectorPose();
  }
}
