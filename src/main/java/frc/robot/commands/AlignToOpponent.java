package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Vision;

public class AlignToOpponent extends CommandBase {
  
  private DrivetrainSubsystem drivetrainSubsystem;
  private Vision vision;
  private DoubleSupplier xSupplier;
  private DoubleSupplier ySupplier;
  private Rotation2d target;

  public AlignToOpponent(Vision vison, DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.vision = vison;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    addRequirements(drivetrainSubsystem, vison);
  }

  @Override
  public void initialize() {

    drivetrainSubsystem.setForceRotationTarget(true);
    target = vision.getBotpose().getRotation().minus(vision.getTargetXOffset()).plus(Rotation2d.fromDegrees(180));

    
    drivetrainSubsystem.setRotationTarget(target);
  }

  @Override
  public void execute() {
    drivetrainSubsystem.resetPose(vision.getBotpose());
    drivetrainSubsystem.drive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), 0);
  }

  @Override
  public boolean isFinished() {
    return !vision.hasTarget() || drivetrainSubsystem.getRotation().equals(target);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.setForceRotationTarget(false);
  }


}
