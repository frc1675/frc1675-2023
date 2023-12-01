// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawIntake extends SubsystemBase {
  /** Creates a new ClawIntake. */
  private boolean isExtended;
  private DoubleSolenoid clawIntake;

  public ClawIntake(int f, int r) {
    this.isExtended = false;
    clawIntake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, f, r);
  }

  public boolean getIsExtended(){
    return this.isExtended;
  }

  public void setIsExtended(boolean extendValue){
    if(extendValue) {
      clawIntake.set(Value.kForward);
    }else {
      clawIntake.set(Value.kReverse);
    }
    this.isExtended = extendValue;
    
  }

}
