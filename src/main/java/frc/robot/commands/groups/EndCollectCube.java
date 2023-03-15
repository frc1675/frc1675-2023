package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.floor.FloorSetSpeed;
import frc.robot.subsystems.FloorArmSubsystem;
import frc.robot.subsystems.FloorIntake;

public class EndCollectCube extends SequentialCommandGroup {

  public EndCollectCube(FloorArmSubsystem floorArm, FloorIntake intake) {
    addCommands(
      new FloorSetSpeed(intake, 0),
      new RetractFloorIntake(floorArm)
    );
  }
}
