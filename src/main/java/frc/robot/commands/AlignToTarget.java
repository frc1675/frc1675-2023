package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Vision;

public class AlignToTarget extends CommandBase {
  
  private DrivetrainSubsystem drivetrainSubsystem;
  private Vision vision;
  private DoubleSupplier xSupplier;
  private DoubleSupplier ySupplier;
  private boolean floorIntakeFacing;
  private Rotation2d target;

  public AlignToTarget(Vision vison, DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier xSupplier, DoubleSupplier ySupplier, boolean floorIntakeFacing) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.vision = vison;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.floorIntakeFacing = floorIntakeFacing;
    addRequirements(drivetrainSubsystem, vison);
  }

  @Override
  public void initialize() {

    drivetrainSubsystem.setForceRotationTarget(true);
    target = vision.getBotpose().getRotation().minus(vision.getTargetXOffset());
    if(floorIntakeFacing) {
      target.plus(Rotation2d.fromDegrees(180));
    }
    
    drivetrainSubsystem.setRotationTarget(target);
  }

  @Override
  public void execute() {
    drivetrainSubsystem.resetPose(vision.getBotpose());
    drivetrainSubsystem.drive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), 0);
  }

  @Override
  public boolean isFinished() {
    return !vision.hasTarget() || drivetrainSubsystem.getGyroscopeRotation().equals(target);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.setForceRotationTarget(false);
  }


}
