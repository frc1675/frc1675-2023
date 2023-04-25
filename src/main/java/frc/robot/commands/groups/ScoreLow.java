package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.floorArm.FloorMoveArmToPostion;
import frc.robot.commands.intake.armIntake.ArmDrop;
import frc.robot.commands.intake.armIntake.ArmIntake;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FloorArmSubsystem;
import frc.robot.subsystems.Intake;

public class ScoreLow extends SequentialCommandGroup {

  public ScoreLow(DrivetrainSubsystem drive, FloorArmSubsystem floorArm, Intake intake, boolean scoreCone) {
    addCommands(
      new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_SHOOTING_POSITION),
      new WaitCommand(0.5),
      new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_INSIDE_ROBOT_POSITION),
      new ConditionalCommand(new ArmIntake(intake).withTimeout(0.5), new ArmDrop(intake).withTimeout(0.5),() -> scoreCone)
    );
  }
}
