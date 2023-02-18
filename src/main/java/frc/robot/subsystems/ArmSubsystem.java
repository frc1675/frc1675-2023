// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax armMotor;
  private Solenoid brake;
  private DutyCycleEncoder encoder;
  private PIDController pid;
  private SparkMaxAbsoluteEncoder absEncoder;
  private double targetPosition;


 


  

  public ArmSubsystem() {
    armMotor = new CANSparkMax(Constants.ARM_MOTOR, MotorType.kBrushless);
    brake = new Solenoid(PneumaticsModuleType.REVPH, Constants.ARM_SOLENOID_CHANNEL);
    pid = new PIDController(Constants.ARM_P_COEFF,Constants.ARM_I_COEFF,Constants.ARM_D_COEFF);
    
    absEncoder = armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.fromId(2));


  }
  
   

  public void moveArm(double power) {
    armMotor.set(power);
  }

  public void lock() {
    brake.set(true);
  }

  public void unlock() {
    brake.set(false);
  }

  public double getPosition() {
    return encoder.getDistance();
  }

  public void setTargetPosition(double position){
    targetPosition = position;
  }

  @Override
  public void periodic() {
    armMotor.set(pid.calculate(getPosition()-targetPosition));
  
  }
}
