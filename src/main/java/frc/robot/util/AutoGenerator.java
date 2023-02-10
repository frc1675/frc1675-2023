package frc.robot.util;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoGenerator {
    private SwerveAutoBuilder builder;
    private DrivetrainSubsystem drivetrainSubsystem;

    private HashMap<String, Command> eventMap = new HashMap<String, Command>();
    private PathConstraints defaulPathConstraints = new PathConstraints(Constants.AUTO_MAX_VELOCITY, Constants.AUTO_MAX_ACCELERATION);

    private SendableChooser<AutoActions> actionSelector = new SendableChooser<AutoActions>();
    private SendableChooser<StartLocation> locationSelector = new SendableChooser<StartLocation>();

    public enum AutoActions {
        EXIT_AND_BALANCE
    } 

    public enum StartLocation {
        ONE(1),
        TWO(2);

        public final int value;
        private StartLocation(int value) {
            this.value = value;
        }
    }

    public AutoGenerator(DrivetrainSubsystem drivetrainSubsystem) {   

        this.drivetrainSubsystem = drivetrainSubsystem;
        SwerveDriveKinematics kinematics = this.drivetrainSubsystem.getKinematics();

        builder = new SwerveAutoBuilder(
            this.drivetrainSubsystem::getPose,
            this.drivetrainSubsystem::resetGyroscope,
            kinematics,
            new PIDConstants(0, 0, 0),
            new PIDConstants(0, 0, 0),
            this.drivetrainSubsystem::setSwerveStates,
            eventMap,
            true,
            this.drivetrainSubsystem
        );

        locationSelector.setDefaultOption("Area One", StartLocation.ONE);
        locationSelector.addOption("Area Two", StartLocation.TWO);

        actionSelector.setDefaultOption("Exit community and balance", AutoActions.EXIT_AND_BALANCE);
    }

    public Command getAutoCommand() {
        if(actionSelector.getSelected() == AutoActions.EXIT_AND_BALANCE) {
            return getExitAndBalance(locationSelector.getSelected());
        }
        return null;
    }

    public Command getExitAndBalance(StartLocation startArea) {
        if(startArea == StartLocation.ONE || startArea == StartLocation.TWO) {
            PathPlannerTrajectory path = PathPlanner.loadPath("ExitAndBalance A" + startArea, defaulPathConstraints);
            eventMap.put("autoBalance", new PrintCommand("Auto balance begin"));
            return builder.fullAuto(path);
        }else {
            return null;
        }
    }

}
