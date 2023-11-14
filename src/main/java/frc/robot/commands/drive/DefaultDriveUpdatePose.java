package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultDriveUpdatePose extends ParallelDeadlineGroup {
  
  public DefaultDriveUpdatePose(DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier translationXSupplier,
  DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier, DoubleSupplier trigger) {
    super(new DefaultDriveCommand(drivetrainSubsystem, translationXSupplier, translationYSupplier, rotationSupplier, trigger));
  }
}
