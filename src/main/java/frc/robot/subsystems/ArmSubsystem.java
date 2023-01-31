// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax armMotor;
  private Solenoid solenoid;
  private DutyCycleEncoder encoder;

  public ArmSubsystem() {
    armMotor = new CANSparkMax(Constants.armMotor, MotorType.kBrushless);
    solenoid = new Solenoid(null, Constants.Solenoid);

    encoder = new DutyCycleEncoder(0);

  }

  public void moveArm(double power) {
    armMotor.set(power);
  }

  public void lock() {
    solenoid.set(true);
  }

  public void unlock() {
    solenoid.set(false);
  }

  public double getPosition() {
    double armEncoderValue = encoder.getDistance();
    return armEncoderValue;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
