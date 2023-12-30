// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm.Arm_Shoulder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Here they go all the constants related to the subsystem and pure
 * software control logic.
 * 
 * Just Software Related for sim
 */
public class Shoulder extends SubsystemBase {
  private static final double GEAR_RATIO = 1;

  private final ShoulderIO shoulderIO;
  private final ShoulderIOInputsAutoLogged inputs = new ShoulderIOInputsAutoLogged(); 

  private SimpleMotorFeedforward shoulderFeedForward;
  private final PIDController shoulderPID;
  private Rotation2d angleSetPoint = null;
  private Double speedSetPoint = 0.0; // Might not be used
  private double lastAngle = 0.0;

  public Shoulder(ShoulderIO shoulderIO) {
    this.shoulderIO = shoulderIO;

    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        shoulderFeedForward = new SimpleMotorFeedforward(0.1, 0.13);
        shoulderPID = new PIDController(0.05, 0.0, 0.0);
        break;
      case SIM:
        shoulderFeedForward = new SimpleMotorFeedforward(0.0, 0.13);
        shoulderPID = new PIDController(0.1, 0.0, 0.0);
        break;
      default:
        shoulderFeedForward = new SimpleMotorFeedforward(0.0, 0.0);
        shoulderPID = new PIDController(0.0, 0.0, 0.0);
        break;
    }
  }

  public void updateInputs(){
    shoulderIO.updateInputs(inputs);
  }

  public void moveToPos(double desiredDegreesAngle){
    double desiredPosRad = Units.degreesToRadians(desiredDegreesAngle);
    shoulderIO.setShoulderDesiredPos(desiredPosRad, 12);//Magic Number
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateInputs();
    Logger.processInputs("Arm/Shoulder", inputs);
  }
}
