package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.floorArm.FloorMoveArmToPostion;
import frc.robot.commands.intake.armIntake.DropCone;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FloorArmSubsystem;
import frc.robot.subsystems.Intake;

public class ExtendAndScoreCone extends SequentialCommandGroup {

  public ExtendAndScoreCone(DrivetrainSubsystem drive, FloorArmSubsystem floorArm, ArmSubsystem arm, Intake intake) {
    addCommands(
      new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_SHOOTING_POSITION),
      new WaitCommand(0.5),
      new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_INSIDE_ROBOT_POSITION),
      new MoveArmToPosition(arm, Constants.ARM_SCORE_HIGH_POSITION),
      new WaitCommand(1),
      new RepeatCommand(new InstantCommand(() -> drive.drive(-1, 0, 0), drive)).withTimeout(0.5),
      new DropCone(intake).withTimeout(0.5),
      new RepeatCommand(new InstantCommand(() -> drive.drive(1, 0, 0), drive)).withTimeout(0.5),
      new MoveArmToPosition(arm, Constants.ARM_INSIDE_ROBOT_POSITION)
    );
  }

}
