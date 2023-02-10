// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax armMotor;
  private Solenoid brake;
  private DutyCycleEncoder encoder;  
  private double x/*place holder */;


  public ArmSubsystem() {
    armMotor = new CANSparkMax(Constants.ARM_MOTOR, MotorType.kBrushless);
    brake = new Solenoid(PneumaticsModuleType.REVPH, Constants.ARM_SOLENOID_CHANNEL);

    
    encoder = new DutyCycleEncoder(0);


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


  @Override
  public void periodic() {
  // Add a conditional
    if(x<Constants.SET_POINT)
    {
      moveArm(0); 
    }
  }
}
