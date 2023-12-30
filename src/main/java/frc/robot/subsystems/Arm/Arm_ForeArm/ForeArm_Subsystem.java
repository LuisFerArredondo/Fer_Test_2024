// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm.Arm_ForeArm;

import static com.revrobotics.CANSparkMax.SoftLimitDirection.*;
import static frc.robot.Constants.ArmConstants.*;
import static lib.StateMachineToolBox.Directions.*;
import static lib.StateMachineToolBox.SubsystemStates.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.StateMachineToolBox.Directions;
import lib.StateMachineToolBox.StateBasedSubsystem;
import lib.StateMachineToolBox.SubsystemStates;

public class ForeArm_Subsystem extends SubsystemBase implements StateBasedSubsystem {
   private CANSparkMax m_motor;
  private SparkMaxPIDController m_pidController;
  // private RelativeEncoder m_encoder;
  private SparkMaxAbsoluteEncoder m_absoluteEncoder;
  private DoubleSolenoid brake;
  private boolean automaticBreakOn = true;

  private Directions directions = IDLE;
  private SubsystemStates subsystemState = HOLDING_POS;

  /** Creates a new ForeArm_Subsystem. */
  public ForeArm_Subsystem() {
   m_motor = new CANSparkMax(Constants.ArmConstants.kShoulder_Id, MotorType.kBrushless);

    m_motor.restoreFactoryDefaults();
    m_motor.setSmartCurrentLimit(kForeArmCurrentLimit);
    m_motor.setInverted(kForeArmIsInverted);
    m_motor.setIdleMode(kForeArmIdleMode);
    m_motor.setSoftLimit(kForward, kForeArmForwardSoftLimit);
    m_motor.setSoftLimit(kReverse, kForeArmReverseSoftLimit);
    m_motor.enableSoftLimit(kForward, kForeArmSofLimitEnabled);
    m_motor.enableSoftLimit(kReverse, kForeArmSofLimitEnabled);
    m_motor.enableVoltageCompensation(kForeArmVoltageCompensation);
    m_motor.setCANTimeout(0); // to not saturate can bus with status messages

    //params: PCM CAN Id, x, PCM slot 1, PCM slot2
    brake = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 0, 1);

    // m_encoder = m_motor.getEncoder();
    m_absoluteEncoder = m_motor.getAbsoluteEncoder(Type.kDutyCycle);
    m_absoluteEncoder.setPositionConversionFactor(
        kShoulderConversionFactor); // Will return angle in rad
    m_absoluteEncoder.setAverageDepth(2); // TODO: -prove: this is resolution 8142 ticks / 2

    m_pidController = m_motor.getPIDController();

    m_pidController.setP(kP_Shoulder, 0);
    m_pidController.setI(kI_Shoulder, 0);
    m_pidController.setD(kD_Shoulder, 0);
    m_pidController.setFF(kFF_Shoulder, 0);
    m_pidController.setIZone(kIz_Shoulder, 0);
    m_pidController.setOutputRange(kMinOutput_Shoulder, kMaxOutput_Shoulder, 0);
    m_pidController.setSmartMotionMaxVelocity(kShoulderMaxVel, 0);
    m_pidController.setSmartMotionMinOutputVelocity(0, 0);
    m_pidController.setSmartMotionMaxAccel(kShoulderMaxAcc, 0);
    m_pidController.setSmartMotionAllowedClosedLoopError(0, 0);// TODO: Check later on

  }

  public void setTargetPos(double rotations) {
    m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
  }

  
  /**
   * @param pose desired pose (Units: Radians)
   */
  public void setTargetPosRotations(double target) {
    m_pidController.setReference(target, ControlType.kSmartMotion);
  }

  // Get encoder pos
  public double getAbsoluteEncoderPose() {
    return m_absoluteEncoder.getPosition();
  }
  
  public void automaticBreak(){//Forward: open - Reverse: Closed
    brake.set(subsystemState == MOVING_TO_POS && automaticBreakOn ? Value.kForward : Value.kReverse);
  }

  public void updateState(boolean isMoving){//TODO: Upgrade this parameter
    subsystemState = isMoving? MOVING_TO_POS: HOLDING_POS;
  }

  public void updateDirection() {
    //TODO:Set Direction based on motor output sign 
    directions = m_motor.get() > 0 ?  UPWARDS : DOWNWARDS;
  }

  public Directions getDirection() {
    return directions;
  }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      automaticBreak();
    }
}
