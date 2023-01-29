package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RotateCommand extends CommandBase{
    private DrivetrainSubsystem drive;
    private Rotation2d target;

    public RotateCommand(DrivetrainSubsystem drive, Rotation2d target) {
        this.drive = drive;
        this.target = target;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.rotate(target);
    }

    @Override
    public boolean isFinished() {
        return withinTolerance();
    }

    private boolean withinTolerance() {
        if(Math.abs(drive.getGyroscopeRotation().minus(target).getDegrees()) <= Constants.DRIVE_ROTATE_TOLERANCE_DEGREES) {
            return true;
        } 

        return false;
    }

    @Override 
    public void end(boolean interrupted) {
        drive.drive(new ChassisSpeeds(0, 0, 0));
    }
}

