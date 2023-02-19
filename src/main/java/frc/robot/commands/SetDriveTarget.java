package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SetDriveTarget extends CommandBase{
    
    private DrivetrainSubsystem drivetrainSubsystem;
    private double target;

    public SetDriveTarget(DrivetrainSubsystem drivetrainSubsystem, double target) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.target = target;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        drivetrainSubsystem.setRotationTarget(Rotation2d.fromDegrees(target));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
