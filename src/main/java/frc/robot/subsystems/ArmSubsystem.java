// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private ShuffleboardTab armTab = Shuffleboard.getTab("ArmSubsystem");
  private CANSparkMax armMotor;
  private ProfiledPIDController pid;
  private SparkMaxAbsoluteEncoder absEncoder;
  private double targetPosition = Constants.ARM_INSIDE_ROBOT_POSITION;

  public ArmSubsystem() {
    armMotor = new CANSparkMax(Constants.ARM_MOTOR, MotorType.kBrushless);
    armMotor.setInverted(true);

    pid = new ProfiledPIDController(Constants.ARM_P_COEFF, Constants.ARM_I_COEFF,Constants.ARM_D_COEFF, new Constraints(Constants.MAX_ARM_VELOCITY, Constants.MAX_ARM_ACCELERATION));
    pid.enableContinuousInput(0, 1);

    setTargetPosition(targetPosition);

    absEncoder = armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    armTab.addNumber("Position", () -> getPosition());
    setSoftLimit();
  }
  
   public void setSoftLimit(){
    armMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    armMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
   }

  public void moveArm(double power) {
    armMotor.set(power);
  }


  public double getPosition() {
    return absEncoder.getPosition();
  }

  public void setTargetPosition(double position){
    targetPosition = position;
    pid.setGoal(targetPosition);
  }

  public double getTargetPosition() {
    return targetPosition;
  }

  @Override
  public void periodic() {
    
    armMotor.set(pid.calculate(getPosition()));
  
  }
}
