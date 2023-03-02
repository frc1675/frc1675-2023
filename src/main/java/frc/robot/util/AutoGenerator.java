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
import frc.robot.commands.groups.ExtendAndScore;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;

public class AutoGenerator {
    private SwerveAutoBuilder builder;
    private DrivetrainSubsystem drivetrainSubsystem;
    private ArmSubsystem armSubsystem;
    private Intake intake;

    private HashMap<String, Command> eventMap = new HashMap<String, Command>();
    private PathConstraints defaulPathConstraints = new PathConstraints(Constants.AUTO_MAX_VELOCITY, Constants.AUTO_MAX_ACCELERATION);

    private SendableChooser<AutoActions> actionSelector = new SendableChooser<AutoActions>();
    private SendableChooser<StartLocation> locationSelector = new SendableChooser<StartLocation>();

    public enum AutoActions {
        SCORE_EXIT_BALANCE,
        SCORE_AND_EXIT
    } 

    public enum StartLocation {
        ONE(1),
        TWO(2);

        public final int value;
        private StartLocation(int value) {
            this.value = value;
        }
    }

    public AutoGenerator(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem, Intake intake) {   

        this.drivetrainSubsystem = drivetrainSubsystem;
        this.armSubsystem = armSubsystem;
        this.intake = intake;
        SwerveDriveKinematics kinematics = this.drivetrainSubsystem.getKinematics();

        builder = new SwerveAutoBuilder(
            this.drivetrainSubsystem::getPose,
            this.drivetrainSubsystem::resetPose,
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

        actionSelector.setDefaultOption("Score, exit community and balance", AutoActions.SCORE_EXIT_BALANCE);
        actionSelector.addOption("Score and exit", AutoActions.SCORE_AND_EXIT);
    }

    public Command getAutoCommand() {
        if(actionSelector.getSelected() == AutoActions.SCORE_EXIT_BALANCE) {
            return getExitAndBalance(locationSelector.getSelected());
        }
        return null;
    }

    public Command getExitAndBalance(StartLocation startArea) {
        if(startArea == StartLocation.ONE || startArea == StartLocation.TWO) {
            PathPlannerTrajectory path = PathPlanner.loadPath("ExitAndBalance A" + startArea.value, defaulPathConstraints);
            eventMap.put("scoreConeHigh", new ExtendAndScore(drivetrainSubsystem, armSubsystem, intake));
            eventMap.put("autoBalance", new PrintCommand("Auto balance begin"));
            return builder.fullAuto(path);
        }else {
            return null;
        }
    }

    public Command getTestPath() {
        PathPlannerTrajectory path = PathPlanner.loadPath("TestPath", defaulPathConstraints);
        return builder.fullAuto(path);
    }

}
