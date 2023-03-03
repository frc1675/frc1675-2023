package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.drive.SetDriveRotationTarget;
import frc.robot.commands.floorArm.FloorMoveArmToPostion;
import frc.robot.commands.intake.floor.FloorDrop;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FloorArmSubsystem;
import frc.robot.subsystems.FloorIntake;

public class RotateAndScoreCube extends SequentialCommandGroup {

  public RotateAndScoreCube(DrivetrainSubsystem drive, FloorArmSubsystem floorArm, ArmSubsystem arm, FloorIntake intake) {
    addCommands(
      new SetDriveRotationTarget(drive, 180),
      new WaitUntilCommand(() -> Math.abs(drive.getRotation().getDegrees() - 180) < 5),
      new SetDriveRotationTarget(drive), 

      new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_SHOOTING_POSITION),
      new WaitCommand(0.5),
      new FloorDrop(intake, Constants.FLOOR_INTAKE_FAST_SPEED).withTimeout(1),
      new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_INSIDE_ROBOT_POSITION),

      new SetDriveRotationTarget(drive, 0),
      new WaitUntilCommand(() -> Math.abs(drive.getRotation().getDegrees()) < 5),
      new SetDriveRotationTarget(drive), 
      new WaitCommand(0.5)
      
    );
  }
}
