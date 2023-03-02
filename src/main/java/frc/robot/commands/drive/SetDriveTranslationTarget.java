package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SetDriveTranslationTarget extends CommandBase{
    
    private DrivetrainSubsystem drivetrainSubsystem;
    private Translation2d target;

    public SetDriveTranslationTarget(DrivetrainSubsystem drivetrainSubsystem, Translation2d target) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.target = target;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        drivetrainSubsystem.setTranslationTarget(target);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
