package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveAtAngle extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;

    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final Rotation2d targetAngle;

    public DriveAtAngle(DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, Rotation2d targetAngle) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.targetAngle = targetAngle;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        drivetrainSubsystem.driveAtAngle(translationXSupplier.getAsDouble(), translationYSupplier.getAsDouble(), targetAngle);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
