// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FloorIntake extends SubsystemBase {
  /** Creates a new FloorIntake. */
  private CANSparkMax intakeMotor = new CANSparkMax( Constants.FLOOR_INTAKE_MOTOR, MotorType.kBrushless);
  public FloorIntake() {

  }

  public void intakeStop(){
    intakeMotor.set(0);
  }

  
  public void intakePickup(double speed){
    intakeMotor.set(speed*-1);
  }

  public void intakeDrop(double speed){
    intakeMotor.set(speed * 1);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
