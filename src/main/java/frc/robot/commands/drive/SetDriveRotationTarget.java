package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SetDriveRotationTarget extends CommandBase{
    
    private DrivetrainSubsystem drivetrainSubsystem;
    private Rotation2d target;

    public SetDriveRotationTarget(DrivetrainSubsystem drivetrainSubsystem, double target) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.target = Rotation2d.fromDegrees(target);
        addRequirements(drivetrainSubsystem);
    }

    public SetDriveRotationTarget(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        drivetrainSubsystem.setRotationTarget(target);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
