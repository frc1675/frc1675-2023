package frc.robot.util;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
    private HashMap<String, Command> startActionMap = new HashMap<String, Command>();
    private PathConstraints defaulPathConstraints = new PathConstraints(Constants.AUTO_MAX_VELOCITY, Constants.AUTO_MAX_ACCELERATION);

    private SendableChooser<PathActions> pathActionSelector = new SendableChooser<PathActions>();
    private SendableChooser<StartActions> startActionSelector = new SendableChooser<StartActions>();
    private SendableChooser<StartLocation> locationSelector = new SendableChooser<StartLocation>();

    private ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    private Field2d field = new Field2d();

    public enum PathActions {
        SCORE_EXIT_BALANCE("ExitAndBalance"),
        SCORE_AND_EXIT("ScoreAndExit");

        public final String value;
        private PathActions(String value) {
            this.value = value + " A";
        }
    } 

    public enum StartActions {
        SCORE_CUBE_LOW("scoreCubeLow"),
        SCORE_CONE_LOW("scoreConeLow"),
        SCORE_CUBE_HIGH("scoreCubeHigh"),
        SCORE_CONE_HIGH("scoreConeHigh");

        public final String value;
        private StartActions(String value) {
            this.value = value;
        }
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

        startActionMap.put("scoreConeLow", new ScoreLow(drivetrainSubsystem, floorArmSubsystem, intake, true));
        startActionMap.put("scoreCubeLow", new ScoreLow(drivetrainSubsystem, floorArmSubsystem, intake, false));
        startActionMap.put("scoreConeHigh", new ExtendAndScoreCone(drivetrainSubsystem, floorArmSubsystem, armSubsystem, intake));
        startActionMap.put("scoreCubeHigh", new RotateAndScoreCube(drivetrainSubsystem, floorArmSubsystem, armSubsystem, floorIntake));

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
        locationSelector.addOption("Area Three", StartLocation.THREE);

        pathActionSelector.setDefaultOption("Score game piece and exit", PathActions.SCORE_AND_EXIT);
        pathActionSelector.addOption("Score cone high, exit community, and balance", PathActions.SCORE_EXIT_BALANCE);

        startActionSelector.setDefaultOption("Score cone low", StartActions.SCORE_CONE_LOW);
        startActionSelector.addOption("Score cube low", StartActions.SCORE_CUBE_LOW);
        startActionSelector.addOption("Score cone high", StartActions.SCORE_CONE_HIGH);
        startActionSelector.addOption("Score cube high", StartActions.SCORE_CUBE_HIGH);

        autoTab.add("Auto Location", locationSelector).withSize(2, 1).withPosition(0, 0);
        autoTab.add("Auto Path Action", pathActionSelector).withSize(3, 1).withPosition(2, 0);
        autoTab.add("Auto Start Action", startActionSelector).withSize(2, 1).withPosition(5, 0);
        autoTab.addString("Current Auto Path: ", () -> getSelectedPath()).withSize(2, 1).withPosition(0, 1);
        autoTab.addString("Current Starting Action: ", () -> getSelectedStartAction()).withSize(2, 1).withPosition(0, 2);
        autoTab.add("Current Auto Trajectory", field).withSize(6, 4).withPosition(2, 1).withProperties(Map.of("colors", "yellow"));
    }

    private String getSelectedPath() {
        return pathActionSelector.getSelected().value + locationSelector.getSelected().value;
    }

    public Command getAutoCommand() {
        return builder.fullAuto(PathPlanner.loadPath(getSelectedPath(), defaulPathConstraints));
    }

    public String getSelectedStartAction() {
        eventMap.put("startAction", startActionMap.get(startActionSelector.getSelected().value));
        return startActionSelector.getSelected().value;
    }

    /* Called periodically while disabled */
    public void updateSelectorPose() {
        PathPlannerTrajectory path = PathPlanner.loadPath(getSelectedPath(), defaulPathConstraints);
        field.setRobotPose(path.getInitialPose());
        field.getObject("traj").setTrajectory(path);
    }
}
