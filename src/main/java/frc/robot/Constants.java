// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

// TODO: Check using arduino to read color values and send them via PWM to DIO Ports on RoboRio
public final class Constants {
    public static final Mode currentMode = Mode.SIM;
    public static final boolean tuningMode = true;

    public static enum Mode {
      /** Running on a real robot. */
      REAL,
  
      /** Running a physics simulator. */
      SIM,
  
      /** Replaying from a log file. */
      REPLAY
    }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ArmConstants {
    // CAN Id's
    public static final byte kShoulder_Id = 4; // NOTE: Magic Number
    public static final byte kForeArm_Id = 5; // NOTE: Magic Number

    // Motor Position for Arm Movements (Units: Revolutions)
    public static final double kShoulderFrontPos = 2 * Math.PI * 0.3; // NOTE: Magic Number
    public static final double kShoulderVerticalPos = 2 * Math.PI * 0; // NOTE: Magic Number

    // They have no units, it all depends on which units the lengths on
    // inverse kinematics are set
    public static final Translation2d kArmPoseForward =
        new Translation2d(5, 9); // NOTE: Magic Numbers
    public static final Translation2d kArmPoseVertical =
        new Translation2d(0, 0); // NOTE: Magic Numbers

    // Stall Current Limit
    public static final int kShoulderCurrentLimit = 40; // Amps
    public static final int kForeArmCurrentLimit = 40; // Amps

    // Is motor Inverted
    public static final boolean kShoulderIsInverted = true;
    public static final boolean kForeArmIsInverted = false;

    // Idle Modes
    public static final IdleMode kShoulderIdleMode = IdleMode.kBrake;
    public static final IdleMode kForeArmIdleMode = IdleMode.kBrake;

    // Soft Limit Config (Units: Radians)
    public static final float kShoulderForwardSoftLimit = (float) (2 * Math.PI);
    public static final float kShoulderReverseSoftLimit = 0;
    public static final float kForeArmForwardSoftLimit = (float) (2 * Math.PI);
    public static final float kForeArmReverseSoftLimit = 0;
    public static final boolean kShoulderSofLimitEnabled = true;
    public static final boolean kForeArmSofLimitEnabled = true;

    // Gear Ratios
    public static final double SHOULDER_GEAR_RATIO = 1;
    public static final double FOREARM_GEAR_RATIO = 1;

    // Absoulte encoders conversion factor (Unit: Radians)
    public static final double kNoConversionFactor = 1;
    public static final double kShoulderConversionFactor =
        (2 * Math.PI) / SHOULDER_GEAR_RATIO; // Gear Ratio 1:1
    public static final double kForeArmConversionFactor =
        (2 * Math.PI) / FOREARM_GEAR_RATIO; // Magic number

    // Voltage Compensation
    public static final double kShoulderVoltageCompensation = 12.0;//Magic number
    public static final double kForeArmVoltageCompensation = 12.0;//Magic number

    //

    // Control Constants
    public static final double kP_Shoulder = 0.1,//Magic number
        kI_Shoulder = 1e-4,//Magic number
        kD_Shoulder = 1,//Magic number
        kIz_Shoulder = 0,//Magic number
        kFF_Shoulder = 0,//Magic number
        kMaxOutput_Shoulder = 1,
        kMinOutput_Shoulder = -1,
        kShoulderMaxRPM = 5700,
        kShoulderMaxVel = 2000,// rpm -SmartMotion Coeff
        kShoulderMaxAcc = 1500;//Smart Motion Coeff

    public static final double kP_ForeArm = 0.1,//Magic number
        kI_ForeArm = 1e-4,//Magic number
        kD_ForeArm = 1,//Magic number
        kIz_ForeArm = 0,//Magic number
        kFF_ForeArm = 0,//Magic number
        kMaxOutput_ForeArm = 1,
        kMinOutput_ForeArm = -1,
        kForeArmMaxRPM = 5700,
        kForeArmMaxVel = 2000,// rpm -SmartMotion Coeff
        kForeArmMaxAcc = 1500;//Smart Motion Coeff
  }
}
