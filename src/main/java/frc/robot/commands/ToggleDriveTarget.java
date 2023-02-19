package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ToggleDriveTarget extends CommandBase{
    
    private DrivetrainSubsystem drivetrainSubsystem;

    public ToggleDriveTarget(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        if(drivetrainSubsystem.getRotationTarget().getDegrees() == 0)  {
            drivetrainSubsystem.setRotationTarget(Rotation2d.fromDegrees(180));
        }else {
            drivetrainSubsystem.setRotationTarget(Rotation2d.fromDegrees(0));
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
