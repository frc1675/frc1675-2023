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
    private PathConstraints defaulPathConstraints = new PathConstraints(Constants.AUTO_MAX_VELOCITY, Constants.AUTO_MAX_ACCELERATION);

    private SendableChooser<AutoActions> actionSelector = new SendableChooser<AutoActions>();
    private SendableChooser<StartLocation> locationSelector = new SendableChooser<StartLocation>();

    private ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    private Field2d field = new Field2d();

    public enum AutoActions {
        SCORE_EXIT_BALANCE("ExitAndBalance"),
        SCORE_CONE_AND_EXIT("ScoreConeAndExit"),
        SCORE_CUBE_AND_EXIT("ScoreCubeAndExit"),
        SCORE_LOW("ScoreLowAndExit");

        public final String value;
        private AutoActions(String value) {
            this.value = value + " A";
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
        locationSelector.addOption("Area Three", StartLocation.THREE);

        actionSelector.setDefaultOption("Score game piece low and exit", AutoActions.SCORE_LOW);
        actionSelector.addOption("Score cube high and exit", AutoActions.SCORE_CUBE_AND_EXIT);
        actionSelector.addOption("Score cone high and exit", AutoActions.SCORE_CONE_AND_EXIT);
        actionSelector.addOption("Score cone high, exit community, and balance", AutoActions.SCORE_EXIT_BALANCE);

        autoTab.add("Auto Location", locationSelector).withSize(2, 1).withPosition(0, 0);
        autoTab.add("Auto Action", actionSelector).withSize(3, 1).withPosition(2, 0);
        autoTab.addString("Current Auto: ", () -> getSelectedAuto()).withSize(2, 1).withPosition(0, 1);
        autoTab.add("Current Auto Trajectory", field).withSize(6, 4).withPosition(2, 1).withProperties(Map.of("colors", "yellow"));
    }

    private String getSelectedAuto() {
        return actionSelector.getSelected().value + locationSelector.getSelected().value;
    }

    public Command getAutoCommand() {
        return builder.fullAuto(PathPlanner.loadPath(getSelectedAuto(), defaulPathConstraints));
    }

    /* Called periodically while disabled */
    public void updateSelectorPose() {
        PathPlannerTrajectory path = PathPlanner.loadPath(getSelectedAuto(), defaulPathConstraints);
        field.setRobotPose(path.getInitialPose());
        field.getObject("traj").setTrajectory(path);
    }
}
