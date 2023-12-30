// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm.Arm_Claw;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw_Subsystem extends SubsystemBase {
  private DoubleSolenoid crab_claw;
  private DoubleSolenoid lobbster_claw;

  public static enum PickItems{
    CONE,
    CUBE,
    RELEASE,
    IDLE
  }
  
  PickItems item = PickItems.IDLE;
  /** Creates a new Claw_Subsystem. */
  public Claw_Subsystem() {
    crab_claw = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 0, 0);
    lobbster_claw = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 0, 0);
  }

  public void automaticPickUp(){
    switch(item){
      case CONE://Close claw
          crab_claw.set(Value.kForward);
          lobbster_claw.set(Value.kForward);
        break;
      case CUBE:
          crab_claw.set(Value.kForward);
          lobbster_claw.set(Value.kReverse);
        break;
      case RELEASE://Open claw
          crab_claw.set(Value.kReverse);
          lobbster_claw.set(Value.kReverse);
        break;

      case IDLE:
      default://Just there
          crab_claw.set(Value.kOff);
          lobbster_claw.set(Value.kOff);
      break;
      }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    automaticPickUp();
  }
}
