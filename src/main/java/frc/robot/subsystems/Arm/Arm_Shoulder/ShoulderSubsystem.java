// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm.Arm_Shoulder;

import static com.revrobotics.CANSparkMax.SoftLimitDirection.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Here goes just pure hardware control, from telling the controller the voltage that
 * should be applied, opt to using the integrated pid controller
 */
public class ShoulderSubsystem extends SubsystemBase implements ShoulderIO {
  /** Creates a new ShoulderSubsystem. */
  CANSparkMax m_motor = new CANSparkMax(0, MotorType.kBrushless);
  SparkMaxAbsoluteEncoder absoluteEncoder = m_motor.getAbsoluteEncoder(Type.kDutyCycle);
  SparkMaxPIDController pidController = m_motor.getPIDController();

  public ShoulderSubsystem() {
    System.out.println("[Init] creating ShoulderIO");

    m_motor.restoreFactoryDefaults();
    m_motor.setSmartCurrentLimit(40);//40 Amps as current limit (2 NM)
    m_motor.setInverted(false);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setSoftLimit(kForward, 2);
    m_motor.setSoftLimit(kReverse, 2);
    m_motor.enableSoftLimit(kForward, true);
    m_motor.enableSoftLimit(kReverse, true);
    m_motor.enableVoltageCompensation(1); //Magic Numbers
    m_motor.setCANTimeout(500); // to not saturate can bus with status messages, we will get warnings dtw about it

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
