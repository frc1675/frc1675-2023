package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.floorArm.FloorMoveArmToPostion;
import frc.robot.subsystems.FloorArmSubsystem;

public class ExtendFloorIntake extends SequentialCommandGroup {

  public ExtendFloorIntake(FloorArmSubsystem floorArm) {
    addCommands(
      new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_GROUND_POSITION),
      new WaitCommand(0.5)
    );
  }
}
