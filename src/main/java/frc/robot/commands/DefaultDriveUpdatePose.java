package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Vision;

public class DefaultDriveUpdatePose extends ParallelDeadlineGroup {
  
  public DefaultDriveUpdatePose(Vision vison, DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier translationXSupplier,
  DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
    super(new DefaultDriveCommand(drivetrainSubsystem, translationXSupplier, translationYSupplier, rotationSupplier));
    addCommands(new VisionPoseUpdate(vison, drivetrainSubsystem));
  }
}
