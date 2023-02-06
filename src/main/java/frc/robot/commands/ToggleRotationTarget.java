package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ToggleRotationTarget extends CommandBase{
    
    private DrivetrainSubsystem drivetrainSubsystem;
    private Rotation2d target;

    public ToggleRotationTarget(DrivetrainSubsystem drivetrainSubsystem, Rotation2d target) {
        this.target = target;
        this.drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        drivetrainSubsystem.toggleRotationTarget(target);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
