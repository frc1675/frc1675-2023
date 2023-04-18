package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
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
    private SlewRateLimiter filterX = new SlewRateLimiter(Constants.SLEW_RATE_LIMIT);
    private SlewRateLimiter filterY = new SlewRateLimiter(Constants.SLEW_RATE_LIMIT);

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


        // Slew-rate limits the forward/backward input, limiting forward/backward acceleration
       
        
        if (x != 0 || y != 0) {
            drivetrainSubsystem.setBalanceTarget(null);
            drivetrainSubsystem.setTranslationTarget(null);
        }
    
        if (rotation != 0) {
            drivetrainSubsystem.setRotationTarget(null);
        }

        if(forceSlow && trigger == 0) {
            drivetrainSubsystem.setSpeeds(filterX.calculate(x)*Constants.SLOW_DRIVE_SCALING, filterY.calculate(y)*Constants.SLOW_DRIVE_SCALING, rotation*Constants.SLOW_DRIVE_SCALING);
        } else {
            drivetrainSubsystem.setSpeeds(filterX.calculate(x), filterY.calculate(y), rotation);
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.setSpeeds(0, 0, 0);
    }
}
