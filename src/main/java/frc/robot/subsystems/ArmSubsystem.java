// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private ShuffleboardTab armTab = Shuffleboard.getTab("ArmSubsystem");
  private CANSparkMax armMotor;
  private PIDController pid;
  private SparkMaxAbsoluteEncoder absEncoder;
  private double targetPosition;


 


  

  public ArmSubsystem() {
    armMotor = new CANSparkMax(Constants.ARM_MOTOR, MotorType.kBrushless);
    pid = new PIDController(Constants.ARM_P_COEFF,Constants.ARM_I_COEFF,Constants.ARM_D_COEFF);
    
    absEncoder = armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.fromId(2));
    armTab.addNumber("Position", () -> getPosition());
    setSoftLimit();
    


  }
  
   public void setSoftLimit(){
    armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    armMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.ARM_MAX_POSITION);
    armMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.ARM_MIN_POSITION);
   }

  public void moveArm(double power) {
    armMotor.set(power);
  }


  public double getPosition() {
    return absEncoder.getPosition();
  }

  public void setTargetPosition(double position){
    targetPosition = position;
  }

  @Override
  public void periodic() {
    
    armMotor.set(pid.calculate(getPosition()-targetPosition));
  
  }
}
