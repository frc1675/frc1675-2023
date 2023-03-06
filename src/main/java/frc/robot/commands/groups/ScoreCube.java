package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.floorArm.FloorMoveArmToPostion;
import frc.robot.commands.intake.floor.FloorDrop;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FloorArmSubsystem;
import frc.robot.subsystems.FloorIntake;

public class ScoreCube extends SequentialCommandGroup {

  public ScoreCube(DrivetrainSubsystem drive, FloorArmSubsystem floorArm, FloorIntake intake) {
    addCommands(
      new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_SHOOTING_POSITION),
      new WaitCommand(0.5),
      new FloorDrop(intake, Constants.FLOOR_INTAKE_FAST_SPEED).withTimeout(1),
      new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_INSIDE_ROBOT_POSITION),
      new WaitCommand(0.5),
      new InstantCommand(drive::zeroGyroscope, drive)
    );
  }
}
