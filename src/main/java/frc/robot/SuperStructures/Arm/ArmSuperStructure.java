// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SuperStructures.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.Arm_Claw.Claw_Subsystem;
import frc.robot.subsystems.Arm.Arm_ForeArm.ForeArm_Subsystem;
import frc.robot.subsystems.Arm.Arm_Shoulder.Shoulder_Old;

/*This class preforms all of the Arm Movements and stores Arm constants*/
public class ArmSuperStructure extends SubsystemBase {
  private Claw_Subsystem claw_Subsystem; 
  private ForeArm_Subsystem foreArm_Subsystem; //These are just the joints
  private Shoulder_Old shoulder_Subsystem;//These are just the joints
  private double l1 = 0;
  private double l2 = 0;

  //TODO: Implement Vision here too

  //Last state is used to avoid the robot having to reset himself to execute a task 
  //it was already doing
  private ArmSuperStructureState lastState;// meant for optimization also
  private ArmSuperStructureState currentState;
  private ArmSuperStructureState desiredState;

  public ArmSuperStructure() {
    claw_Subsystem = new Claw_Subsystem();
    foreArm_Subsystem = new ForeArm_Subsystem();
    shoulder_Subsystem = new Shoulder_Old();
  }

  public void setMode(ArmSuperStructureState.Arm_Modes wantedMode){
    desiredState.setWantedMode(wantedMode);
    currentState.setWantedMode(desiredState.getWantedMode());
  }

  public Rotation2d[] getAngleInverseKinematics(Translation2d desiredPose) {
    // Angle Fore Arm
    double q2 = -Math.acos((Math.pow(desiredPose.getX(), 2) + Math.pow(desiredPose.getY(), 2) - Math.pow(l1, 2) - Math.pow(l2, 2)) / (2 * l1 * l2));
    // Angle Shoulder
    double q1 = Math.atan(desiredPose.getY() / desiredPose.getX()) + Math.atan((l2 * Math.sin(q2)) / (l1 + l2 * Math.cos(q2)));
    Rotation2d[] angles = {new Rotation2d(q1), new Rotation2d(q2)};//Radians angles

    return angles;
  }

  public Translation2d getCurrentPoseInverseKinematics(){
    /**
     * double[] pose = {
     * l2*Math.cos(q1 + q2) + l1*Math.cos(q1),
     * l2*Math.sin(q1 + q2) + l1*Math.sin(q1)
     * }
     */
    //TODO: fix the mess you have with the angles of the motors, make them equal(Shoulder and fore arm's code)
    return null;
  }

  public void allowArmMovement(){
    //TODO: check that the motors work in rotations not in radians, or change the getMethods 
    //in theory, this will avoid the constant repetition of calculating the angles
    //This based on the flow of the command scheduler
    if(lastState.getWantedMode() != currentState.getWantedMode()){
      Rotation2d[] angles = getAngleInverseKinematics(currentState.getDesiredPos());
      shoulder_Subsystem.setTargetPosRotations(angles[0].getRotations());
      foreArm_Subsystem.setTargetPos(angles[1].getRotations());
    }   
    
  }   
     
  //TODO: Check the logic of this one better, cause it might caue jaming
  private void updateSubsystemsStates(){
    shoulder_Subsystem.updateState(!currentState.isInRange());
    foreArm_Subsystem.updateState(!currentState.isInRange());
  }

  private void updateArmPos(){
    currentState.setArmCurrentPos(getCurrentPoseInverseKinematics());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //NOTE: The periodic method is first called in the command scheduler flow
    updateArmPos();
    currentState.updateState();
    updateSubsystemsStates();

    
    if(!currentState.areEqual(lastState)){
     lastState = currentState;
     //TODO: Log this change or display it somewhere
    }

    if(!lastState.areEqual(desiredState)){
     currentState.updateState();
    }
   }
}

    /*
    Here the states are supposed to change this way: 
     -desiredState -> it stores the mode commanded to the superstructure and the final state the robot should be at
     -currentState -> it changes depending on the actions of the robot trying to reach the state of the "desiredState"
     -lastState -> keeps track of the previous state of the "currentState" and the mode the robot was commanded to, stored in "desiredState"

     So basically the robot should keep track of its future, present and past
    **/