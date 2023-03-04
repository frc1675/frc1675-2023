package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.floorArm.FloorMoveArmToPostion;
import frc.robot.commands.intake.floor.FloorPickup;
import frc.robot.subsystems.FloorArmSubsystem;
import frc.robot.subsystems.FloorIntake;

public class CollectCube extends SequentialCommandGroup {

  public CollectCube(FloorArmSubsystem floorArm, FloorIntake intake) {
    addCommands(
      new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_GROUND_POSITION),
      new WaitCommand(0.5),
      new FloorPickup(intake).withTimeout(1),
      new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_INSIDE_ROBOT_POSITION)
    );
  }
}
