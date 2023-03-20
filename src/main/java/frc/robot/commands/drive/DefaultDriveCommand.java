package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;

    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;
    private final DoubleSupplier triggerSupplier;
    private final BooleanSupplier forceSlowSupplier;

    private double x;
    private double y;
    private double rotation;
    private double trigger;
    private boolean forceSlow;

    private double[] rollingInputX = new double[Constants.INPUT_ROLLING_AVERAGE_SAMPLE_SIZE];
    private double[] rollingInputY = new double[Constants.INPUT_ROLLING_AVERAGE_SAMPLE_SIZE];
    private int index = 0;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier, DoubleSupplier triggerSupplier, BooleanSupplier forceSlowSupplier) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;
        this.triggerSupplier = triggerSupplier;
        this.forceSlowSupplier = forceSlowSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        x = translationXSupplier.getAsDouble();
        y = translationYSupplier.getAsDouble();
        rotation = rotationSupplier.getAsDouble()*Constants.SLOW_ROTATION_SCALING;
        trigger = triggerSupplier.getAsDouble();
        forceSlow = forceSlowSupplier.getAsBoolean();

        if (x != 0 || y != 0) {
            drivetrainSubsystem.setBalanceTarget(null);
            drivetrainSubsystem.setTranslationTarget(null);
        }
    
        if (rotation != 0) {
            drivetrainSubsystem.setRotationTarget(null);
        }
        rollingInputX[index] = x;
        rollingInputY[index] = y;
        index++;
        if(index == Constants.INPUT_ROLLING_AVERAGE_SAMPLE_SIZE) {
            index = 0;
        }

        if(forceSlow && trigger == 0) {
            drivetrainSubsystem.drive(getAverage(rollingInputX) * Constants.SLOW_DRIVE_SCALING, getAverage(rollingInputY) * Constants.SLOW_DRIVE_SCALING, rotation * Constants.SLOW_DRIVE_SCALING);
        } else {
            drivetrainSubsystem.drive(getAverage(rollingInputX), getAverage(rollingInputY), rotation);
        }
        
    }

    private double getAverage(double[] arr) {
        double rtn = 0;
        for(double i : arr) {
            rtn+=i;
        }
        return rtn / arr.length;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
