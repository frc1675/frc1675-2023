package frc.robot.commands.groups;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.drive.SetDriveTranslationTarget;
import frc.robot.commands.floorArm.FloorMoveArmToPostion;
import frc.robot.commands.intake.armIntake.DropCone;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FloorArmSubsystem;
import frc.robot.subsystems.Intake;

public class ExtendAndScoreCone extends SequentialCommandGroup {

  private Supplier<Pose2d> startingPoseSupplier;

  public ExtendAndScoreCone(DrivetrainSubsystem drive, FloorArmSubsystem floorArm, ArmSubsystem arm, Intake intake, Supplier<Pose2d> startingPoseSupplier) {
    this.startingPoseSupplier = startingPoseSupplier;
    addCommands(
      new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_SHOOTING_POSITION),
      new WaitCommand(0.5),
      new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_INSIDE_ROBOT_POSITION),
      new MoveArmToPosition(arm, Constants.ARM_SCORE_HIGH_POSITION),
      new WaitCommand(1),
      new SetDriveTranslationTarget(drive, getTranslationTarget(-0.2)),
      new DropCone(intake).withTimeout(0.5),
      new SetDriveTranslationTarget(drive, getTranslationTarget(0.2)),
      new MoveArmToPosition(arm, Constants.ARM_INSIDE_ROBOT_POSITION)
    );
  }

  private Translation2d getTranslationTarget(double dx) {
    return startingPoseSupplier.get().getTranslation().plus(new Translation2d(dx, 0));
  }
}
