package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.vision.VisionPoseUpdate;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Vision;

public class UpdatePoseParallel extends ParallelDeadlineGroup {

  public UpdatePoseParallel(Command deadlineCommand, Vision vision, DrivetrainSubsystem drivetrainSubsystem) {
    super(deadlineCommand);
    addCommands(new VisionPoseUpdate(vision, drivetrainSubsystem));
  }
}
