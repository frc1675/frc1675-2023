package frc.robot.commands.groups;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.drive.SetDriveTranslationTarget;
import frc.robot.commands.intake.armIntake.DropCone;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;

public class ExtendAndScore extends SequentialCommandGroup {

  public ExtendAndScore(DrivetrainSubsystem drive, ArmSubsystem arm, Intake intake) {
    addCommands(
      new MoveArmToPosition(arm, Constants.ARM_SCORE_HIGH_POSITION),
      new SetDriveTranslationTarget(drive, getTranslationTarget(drive, 1)),
      //new WaitUntilCommand(() -> Math.abs(arm.getPosition() - arm.getTargetPosition()) < Constants.ARM_ENCODER_COUNT_ERROR),
      new WaitCommand(1),
      new DropCone(intake).withTimeout(1),//could probably tune this down
      new SetDriveTranslationTarget(drive, getTranslationTarget(drive, -1)),
      new MoveArmToPosition(arm, Constants.ARM_INSIDE_ROBOT_POSITION)
    );
  }

  private Translation2d getTranslationTarget(DrivetrainSubsystem drive, double dx) {
    return drive.getPose().getTranslation().plus(new Translation2d(dx, 0));
  }
}
