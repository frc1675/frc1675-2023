package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.intake.armIntake.DropCone;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Intake;

public class ExtendAndScore extends SequentialCommandGroup {

  public ExtendAndScore(ArmSubsystem arm, Intake intake) {
    addCommands(
      new MoveArmToPosition(arm, Constants.ARM_SCORE_HIGH_POSITION),
      //translate forward
      new WaitUntilCommand(() -> Math.abs(arm.getPosition() - arm.getTargetPosition()) < Constants.ARM_ENCODER_COUNT_ERROR),
      new DropCone(intake).withTimeout(1),//could probably tune this down
      //translate away
      new MoveArmToPosition(arm, Constants.ARM_INSIDE_ROBOT_POSITION)
    );
  }
}
