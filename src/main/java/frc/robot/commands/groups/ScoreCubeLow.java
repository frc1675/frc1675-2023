package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.floorArm.FloorMoveArmToPostion;
import frc.robot.commands.intake.armIntake.DropCube;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FloorArmSubsystem;
import frc.robot.subsystems.Intake;

public class ScoreCubeLow extends SequentialCommandGroup {

  public ScoreCubeLow(DrivetrainSubsystem drive, FloorArmSubsystem floorArm, Intake intake) {
    addCommands(
      new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_SHOOTING_POSITION),
      new WaitCommand(0.5),
      new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_INSIDE_ROBOT_POSITION),
      new DropCube(intake).withTimeout(0.5)
    );
  }
}
