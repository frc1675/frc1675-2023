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
import frc.robot.subsystems.FloorArmSubsystem;
import frc.robot.subsystems.Intake;

public class AutoGenerator {
    private SwerveAutoBuilder builder;

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

    public AutoGenerator(DrivetrainSubsystem drivetrainSubsystem, FloorArmSubsystem floorArmSubsystem, ArmSubsystem armSubsystem, Intake intake) {   
        SwerveDriveKinematics kinematics = drivetrainSubsystem.getKinematics();

        builder = new SwerveAutoBuilder(
            drivetrainSubsystem::getPose,
            drivetrainSubsystem::resetPose,
            kinematics,
            new PIDConstants(0, 0, 0),
            new PIDConstants(0, 0, 0),
            drivetrainSubsystem::setSwerveStates,
            eventMap,
            true,
            drivetrainSubsystem
        );

        locationSelector.setDefaultOption("Area One", StartLocation.ONE);
        locationSelector.addOption("Area Two", StartLocation.TWO);

        actionSelector.setDefaultOption("Score and exit", AutoActions.SCORE_AND_EXIT);
        actionSelector.addOption("Score, exit community and balance", AutoActions.SCORE_EXIT_BALANCE);

        eventMap.put("scoreConeHigh", new ExtendAndScore(drivetrainSubsystem, floorArmSubsystem, armSubsystem, intake));
        eventMap.put("autoBalance", new PrintCommand("Auto balance begin"));
    }

    public Command getAutoCommand() {
        if(actionSelector.getSelected() == AutoActions.SCORE_EXIT_BALANCE) {
            System.out.print("Auto: Score, exit, and balance");
            return getExitAndBalance(locationSelector.getSelected());
        }else if(actionSelector.getSelected() == AutoActions.SCORE_AND_EXIT) {
            System.out.print("Auto: Score and exit");
            return getScoreAndExit(locationSelector.getSelected());
        }
        return null;
    }

    public Command getExitAndBalance(StartLocation startArea) {
        if(startArea == StartLocation.ONE || startArea == StartLocation.TWO) {
            PathPlannerTrajectory path = PathPlanner.loadPath("ExitAndBalance A" + startArea.value, defaulPathConstraints);
            System.out.println(" (Area " + startArea.value + " )");
            return builder.fullAuto(path);
        }else {
            return null;
        }
    }

    public Command getScoreAndExit(StartLocation startArea) {
        if(startArea == StartLocation.ONE || startArea == StartLocation.TWO) {
            PathPlannerTrajectory path = PathPlanner.loadPath("ScoreAndExit A" + startArea.value, defaulPathConstraints);
            System.out.println(" (Area " + startArea.value + " )");
            return builder.fullAuto(path);
        }else {
            return null;
        }
    }

}
