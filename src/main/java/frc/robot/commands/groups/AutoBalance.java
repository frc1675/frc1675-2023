package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.floorArm.FloorMoveArmToPostion;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FloorArmSubsystem;

public class AutoBalance extends SequentialCommandGroup {

  public AutoBalance(DrivetrainSubsystem drive, FloorArmSubsystem floorArm) {
    addCommands(
      new RepeatCommand(
        new SequentialCommandGroup(
          new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_SHOOTING_POSITION),
          new InstantCommand(() -> drive.drive(1, 0, 0)),
          new WaitCommand(0.15),
          new FloorMoveArmToPostion(floorArm, Constants.FLOOR_ARM_GROUND_POSITION),
          new WaitCommand(0.65)
        )
      ).until(
            () -> drive.getGyroscopePitch().getDegrees() >= Constants.AUTO_BALANCE_ENGAGE_DEGREES
            || drive.getGyroscopeRoll().getDegrees() >= Constants.AUTO_BALANCE_ENGAGE_DEGREES
      ),
      new InstantCommand(()-> drive.setBalanceTargetDefault(), drive),
      new WaitUntilCommand(
        () -> drive.getGyroscopePitch().getDegrees() <= Constants.AUTO_BALANCE_TOLERANCE_DEGREES
        && drive.getGyroscopeRoll().getDegrees() <= Constants.AUTO_BALANCE_TOLERANCE_DEGREES
      ),
      new InstantCommand(() -> drive.setBalanceTarget(null)),
      new StartEndCommand(
        () -> drive.drive(0,0,1),
        ()-> drive.drive(0, 0, 0), 
        drive
      ).withTimeout(0.01)
    );
  }

}
