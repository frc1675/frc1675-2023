package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ToggleRotationTarget extends CommandBase{
    
    private DrivetrainSubsystem drivetrainSubsystem;
    private Rotation2d target;

    public ToggleRotationTarget(DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier targetDegrees) {
        this.target = Rotation2d.fromDegrees(targetDegrees.getAsDouble());
        this.drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        if(drivetrainSubsystem.getRotationTargetEnabled()) {
            drivetrainSubsystem.disableRotationTarget();
        }else {
            drivetrainSubsystem.enableRotationTarget(target);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
