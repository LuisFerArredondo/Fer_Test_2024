// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm.Arm_Shoulder;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/**
   *  Here they go all the variables related to the idea of the subsystem
  */
public interface ShoulderIO {

    @AutoLog
    public static class ShoulderIOInputs{
        public double shoulderRelativePosRad = 0.0;
        public double shoulderAbsoultePosRad = 0.0;
        public double shoulderAppliedVolts = 0.0;
        public boolean shoulderBreakActive = false;
        public double[] shoulderCurrentAmps = new double[] {};

        public Rotation2d angleAbsolutePos = new Rotation2d();
        public Rotation2d angleRelativePos = new Rotation2d();
    }
    
  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShoulderIOInputs inputs) {}

  /** Run the drive motor at the specified voltage. */
  public default void setShoulderVoltage(double volts) {}

  /** */
  public default void setPidConstants(double p, double i, double d, double ff){}

  /** Drive the motor at the specified position. */
  public default void setShoulderDesiredPos(double posInRad, double ffVolts) {}

  /** Enable or disable brake mode on the main motor. */
  public default void setShoulderMotorBrakeMode(boolean enable) {}

  /** Enable the gearbox built-in solenoid brake  */
  public default void setShoulderBrakeOn(boolean enable) {}
}
