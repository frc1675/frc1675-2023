// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private ShuffleboardTab armIntakeTab = Shuffleboard.getTab("Intake");
  private CANSparkMax intakeMotor = new CANSparkMax( Constants.INTAKE_MOTOR, MotorType.kBrushless);
  private CANSparkMax inverseIntakeMotor = new CANSparkMax(Constants.INVERSE_INTAKE_MOTOR, MotorType.kBrushless);

  public Intake() {
    intakeMotor.setInverted(false);
    inverseIntakeMotor.setInverted(true);
    inverseIntakeMotor.follow(intakeMotor);
    armIntakeTab.addNumber("Current", () -> getCurrent());
    intakeMotor.setSmartCurrentLimit(Constants.INTAKE_CURRENT_LIMIT);  
    inverseIntakeMotor.setSmartCurrentLimit(Constants.INTAKE_CURRENT_LIMIT);
  }

  public double getCurrent(){
    return intakeMotor.getOutputCurrent();
  }

  public void intakeStop(){
    intakeMotor.set(0);
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }
}
