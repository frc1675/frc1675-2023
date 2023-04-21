package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoGenerator;

public class TrajectoryDrive extends CommandBase {
    private final DrivetrainSubsystem drive;
    private final AutoGenerator autoGenerator;
    private final Pose2d endpoint;
    

    private Command c;

    public TrajectoryDrive(AutoGenerator autoGenerator, DrivetrainSubsystem drivetrainSubsystem, Pose2d endpoint) {
        this.autoGenerator = autoGenerator;
        this.drive = drivetrainSubsystem;
        this.endpoint = endpoint;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        c = autoGenerator.generateTrajectory(endpoint);
        CommandScheduler.getInstance().schedule(c);
    }

    @Override
    public boolean isFinished() {
        return c.isFinished();
    }

    @Override
    public void end(boolean i) {
        if(c.isScheduled()) {
            c.cancel();
        }
        drive.setSpeeds(0, 0, 0);
    }
}
