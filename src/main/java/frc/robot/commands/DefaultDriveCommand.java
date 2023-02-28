package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
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
        rotation = rotationSupplier.getAsDouble();
        trigger = triggerSupplier.getAsDouble();
        forceSlow = forceSlowSupplier.getAsBoolean();
    
        
        //if(activateSlowDrive(drivetrainSubsystem.getPose()) && trigger == 0 || forceSlow && trigger == 0) {
        if(forceSlow && trigger == 0) {
            drivetrainSubsystem.drive(x * Constants.SLOW_DRIVE_SCALING, y * Constants.SLOW_DRIVE_SCALING, rotation * Constants.SLOW_DRIVE_SCALING);
        } else {
            drivetrainSubsystem.drive(x, y, rotation);
        }

        if (rotation != 0) {
            drivetrainSubsystem.setRotationTarget(null);
        }
    }

    private boolean activateSlowDrive(Pose2d pose) {
        return withinCommunity(pose) || withinHumanPlayer(pose);
    }

    private boolean withinCommunity(Pose2d pose) {
        if(pose.getX() > Constants.COMMUNITY_MAX_WIDTH_METERS) {
            return false;
        }

        if(pose.getY() > Constants.COMMUNITY_HEIGHT_METERS) {
            return false; //really should not happen but not impossible
        }

        if(pose.getX() > Constants.COMMUNITY_MIN_WIDTH_METERS && pose.getY() > Constants.COMMUNITY_MAX_WIDTH_HEIGHT_METERS) {
            return false;
        }

        return true;
    }

    private boolean withinHumanPlayer(Pose2d pose) {
        if(pose.getY() < Constants.FIELD_HEIGHT_METERS - Constants.HUMAN_PLAYER_HEIGHT_METERS) {
            return false;
        }

        if(pose.getX() < Constants.FIELD_WIDTH_METERS - Constants.HUMAN_PLAYER_MAX_WIDTH_METERS) {
            return false;
        }

        if(pose.getX() < Constants.FIELD_WIDTH_METERS - Constants.HUMAN_PLAYER_MIN_WIDTH_METERS && pose.getY() < Constants.FIELD_HEIGHT_METERS - Constants.HUMAN_PLAYER_MIN_WIDTH_HEIGHT_METERS) {
            return false;
        }
        
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
