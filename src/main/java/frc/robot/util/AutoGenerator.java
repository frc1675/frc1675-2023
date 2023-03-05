package frc.robot.util;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.commands.groups.BeginCollectCube;
import frc.robot.commands.groups.EndCollectCube;
import frc.robot.commands.groups.ExtendAndScoreCone;
import frc.robot.commands.groups.RotateAndScoreCube;
import frc.robot.commands.groups.ScoreLow;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FloorArmSubsystem;
import frc.robot.subsystems.FloorIntake;
import frc.robot.subsystems.Intake;

public class AutoGenerator {
    private SwerveAutoBuilder builder;

    private HashMap<String, Command> eventMap = new HashMap<String, Command>();
    private PathConstraints defaulPathConstraints = new PathConstraints(Constants.AUTO_MAX_VELOCITY, Constants.AUTO_MAX_ACCELERATION);

    private SendableChooser<AutoActions> actionSelector = new SendableChooser<AutoActions>();
    private SendableChooser<StartLocation> locationSelector = new SendableChooser<StartLocation>();

    private ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");

    public enum AutoActions {
        SCORE_EXIT_BALANCE,
        SCORE_CONE_AND_EXIT,
        SCORE_CUBE_AND_EXIT,
        SCORE_LOW
    } 

    public enum StartLocation {
        ONE(1),
        THREE(3);

        public final int value;
        private StartLocation(int value) {
            this.value = value;
        }
    }

    public AutoGenerator(DrivetrainSubsystem drivetrainSubsystem, FloorArmSubsystem floorArmSubsystem, ArmSubsystem armSubsystem, Intake intake, FloorIntake floorIntake) {   
        SwerveDriveKinematics kinematics = drivetrainSubsystem.getKinematics();

        eventMap.put("scoreLow", new ScoreLow(drivetrainSubsystem, floorArmSubsystem, intake));
        eventMap.put("scoreConeHigh", new ExtendAndScoreCone(drivetrainSubsystem, floorArmSubsystem, armSubsystem, intake));
        eventMap.put("scoreCubeHigh", new RotateAndScoreCube(drivetrainSubsystem, floorArmSubsystem, armSubsystem, floorIntake));
        eventMap.put("autoBalance", new PrintCommand("Auto balance begin"));
        eventMap.put("beginCollectCube", new BeginCollectCube(floorArmSubsystem, floorIntake));
        eventMap.put("endCollectCube", new EndCollectCube(floorArmSubsystem, floorIntake));

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
        locationSelector.addOption("Area Two", StartLocation.THREE);

        actionSelector.setDefaultOption("Score something low and exit", AutoActions.SCORE_LOW);
        actionSelector.addOption("Score cube high and exit", AutoActions.SCORE_CUBE_AND_EXIT);
        actionSelector.addOption("Score cone high and exit", AutoActions.SCORE_CONE_AND_EXIT);
        actionSelector.addOption("Score cone high, exit community, and balance", AutoActions.SCORE_EXIT_BALANCE);

        autoTab.add("Auto Location", locationSelector).withSize(2, 1);
        autoTab.add("Auto Action", actionSelector).withSize(2, 1);
    }

    public Command getAutoCommand() {
        if(actionSelector.getSelected() == AutoActions.SCORE_EXIT_BALANCE) {
            DriverStation.reportWarning("Auto: Score cone high, exit, and balance", false);
            return getExitAndBalance(locationSelector.getSelected());

        }else if(actionSelector.getSelected() == AutoActions.SCORE_CONE_AND_EXIT) {
            DriverStation.reportWarning("Auto: Score cone high and exit", false);
            return getScoreConeAndExit(locationSelector.getSelected());

        }else if(actionSelector.getSelected() == AutoActions.SCORE_CUBE_AND_EXIT) {
            DriverStation.reportWarning("Auto: Score cube high and exit", false);
            return getScoreCubeAndExit(locationSelector.getSelected());

        }else if(actionSelector.getSelected() == AutoActions.SCORE_LOW) {
            DriverStation.reportWarning("Auto: score something low and exit", false);
            return getScoreLowAndExit(locationSelector.getSelected());
        }
        return null;
    }

    public Command getExitAndBalance(StartLocation startArea) {
        if(startArea == StartLocation.ONE || startArea == StartLocation.THREE) {
            PathPlannerTrajectory path = PathPlanner.loadPath("ExitAndBalance A" + startArea.value, defaulPathConstraints);
            DriverStation.reportWarning("Auto Area: " + startArea.value, false);
            return builder.fullAuto(path);
        }else {
            return null;
        }
    }

    public Command getScoreConeAndExit(StartLocation startArea) {
        if(startArea == StartLocation.ONE || startArea == StartLocation.THREE) {
            PathPlannerTrajectory path = PathPlanner.loadPath("ScoreConeAndExit A" + startArea.value, defaulPathConstraints);
            DriverStation.reportWarning("Auto Area: " + startArea.value, false);
            return builder.fullAuto(path);
        }else {
            return null;
        }
    }

    public Command getScoreCubeAndExit(StartLocation startArea) {
        if(startArea == StartLocation.ONE || startArea == StartLocation.THREE) {
            PathPlannerTrajectory path = PathPlanner.loadPath("ScoreCubeAndExit A" + startArea.value, defaulPathConstraints);
            DriverStation.reportWarning("Auto Area: " + startArea.value, false);
            return builder.fullAuto(path);
        }else {
            return null;
        }
    }

    public Command getScoreLowAndExit(StartLocation startArea) {
        if(startArea == StartLocation.ONE || startArea == StartLocation.THREE) {
            PathPlannerTrajectory path = PathPlanner.loadPath("ScoreLowAndExit A" + startArea.value, defaulPathConstraints);
            DriverStation.reportWarning("Auto Area: " + startArea.value, false);
            return builder.fullAuto(path);
        }else {
            return null;
        }
    }

}
