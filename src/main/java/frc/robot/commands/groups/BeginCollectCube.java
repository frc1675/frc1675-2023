package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.intake.floor.FloorSetSpeed;
import frc.robot.subsystems.FloorArmSubsystem;
import frc.robot.subsystems.FloorIntake;

public class BeginCollectCube extends SequentialCommandGroup {

  public BeginCollectCube(FloorArmSubsystem floorArm, FloorIntake intake) {
    addCommands(
      new ExtendFloorIntake(floorArm),
      new FloorSetSpeed(intake, Constants.INTAKE_SPEED)
      
    );
  }
}
