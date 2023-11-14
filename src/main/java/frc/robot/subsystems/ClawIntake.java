// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawIntake extends SubsystemBase {
  /** Creates a new ClawIntake. */
  private boolean isExtended;
  private Solenoid clawIntake;

  public ClawIntake(PneumaticsModuleType moduleType, int channel) {
    this.isExtended = false;
    clawIntake = new Solenoid(moduleType, channel);
  }

  public boolean getIsExtended(){
    return this.isExtended;
  }

  public void setIsExtended(boolean extendValue){
    this.isExtended = extendValue;
  }

  public int getChannel(){
    return this.clawIntake.getChannel();
  }

  public void endSolenoidConnection(){
    this.clawIntake.close();
  }

  @Override
  public void periodic() {
    this.clawIntake.set(this.isExtended);
  }
}
