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

public class FloorArmSubsystem extends SubsystemBase {
  private ShuffleboardTab floorArmTab = Shuffleboard.getTab("FloorArm");
  private CANSparkMax floorArmMotor;
  private PIDController pid;
  private SparkMaxAbsoluteEncoder absEncoder;
  private double targetPosition = Constants.FLOOR_ARM_INSIDE_ROBOT_POSITION;

  public FloorArmSubsystem() {
    floorArmMotor = new CANSparkMax(Constants.FLOOR_ARM_MOTOR, MotorType.kBrushless);
    floorArmMotor.setInverted(false);
    pid = new PIDController(Constants.FLOOR_ARM_P_COEFF,Constants.FLOOR_ARM_I_COEFF,Constants.FLOOR_ARM_D_COEFF);
    pid.enableContinuousInput(0, 1);
    setTargetPosition(targetPosition);
    absEncoder = floorArmMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    floorArmTab.addNumber("Position", () -> getPosition());
    setSoftLimit();
  }
  
   public void setSoftLimit(){
    floorArmMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    floorArmMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
   }

  public void moveArm(double power) {
    floorArmMotor.set(power);
  }


  public double getPosition() {
    return absEncoder.getPosition();
  }

  public void setTargetPosition(double position){
    targetPosition = position;
    pid.setSetpoint(targetPosition);
  }

  public double getTargetPosition() {
    return targetPosition;
  }

  @Override
  public void periodic() {
    floorArmMotor.set(pid.calculate(getPosition()));
  }
}
