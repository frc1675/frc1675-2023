package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;

    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    private final PIDController pid = new PIDController(Constants.PROPORTIONAL_COEFFICENT,
            Constants.INTEGRAL_COEFFICENT, Constants.DERIVATIVE_COEFFICENT);
    private Rotation2d target;
    private boolean updateTarget = true;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        updateTarget();
        drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationXSupplier.getAsDouble(),
                        translationYSupplier.getAsDouble(),
                        getRotation(),
                        drivetrainSubsystem.getGyroscopeRotation()));
    }

    private void updateTarget() {
        if (rotationSupplier.getAsDouble() == 0) {
            if (updateTarget) {
                target = drivetrainSubsystem.getGyroscopeRotation();
                updateTarget = false;
            }
        } else {
            updateTarget = true;
        }

    }

    private double getRotation() {
        if (rotationSupplier.getAsDouble() == 0
                && (translationXSupplier.getAsDouble() != 0 || translationYSupplier.getAsDouble() != 0)
                && target != null) {
            return -pid.calculate(drivetrainSubsystem.getGyroscopeRotation().minus(target).getRadians());
        } else {
            return rotationSupplier.getAsDouble();
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
