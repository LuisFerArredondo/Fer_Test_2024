// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SuperStructures.Arm;

import edu.wpi.first.math.geometry.Translation2d;

/** 
 * This class stores all of the possible states and modes of the Arm Superstructure 
 * Also tools that can become handy with states, the states are meant to be automatically changed 
 * with just one method, not manually set in the code.
 * 
 * The states of the superstructures are meant for a more schematic code arquitecture and 
 * as a communication method between other superstructures and drivers.
 * */
public class ArmSuperStructureState {
    private Arm_Modes wantedMode = Arm_Modes.IDLE;
    private Arm_States state = Arm_States.IDLE;
    private boolean isWantedState = false;
    private boolean isInRange = false;  
    private Translation2d currentArmPos;
    //Switch
    public static enum Arm_Modes{
        PICK_ITEMS_POS(new Translation2d(0,0)),
        SCORE_HIGH_GOAL(new Translation2d(0,0)),
        SCORE_MID_GOAL(new Translation2d(0,0)),
        SCORE_LOW_GOAL(new Translation2d(0,0)),
        IDLE(new Translation2d());

        Translation2d pose2d;
        Arm_Modes(Translation2d pose){
            pose2d = pose;
        }

        public Translation2d getDesiredPose(){
            return pose2d;
        }

      }

    public static enum Arm_States{
        MOVING_TO_HIGH,
        MOVING_TO_MID,
        MOVING_TO_LOW,
        MOVING_TO_PICK,
        IN_HIGH_POS,
        IN_MID_POS,
        IN_LOW_POS,
        IN_PICK_POS,
        IDLE
    }
    
    public ArmSuperStructureState(){}
    
    //Mode methods 
    public void setWantedMode(Arm_Modes wantedMode){
        this.wantedMode = wantedMode;
    }
    
    public Arm_Modes getWantedMode(){
        return wantedMode;
    }    

    //State Methods
    public ArmSuperStructureState wantedState(){
        isWantedState = true;
        return this;
    }

    public Arm_States getState(){
        return state;
    }

    public Arm_States getTargetState(){
        Arm_States returnState;
        switch(wantedMode){
            case PICK_ITEMS_POS:
                    returnState = Arm_States.IN_PICK_POS;
                break;
            case SCORE_HIGH_GOAL:
                    returnState = Arm_States.IN_HIGH_POS;
                break;
            case SCORE_MID_GOAL:
                    returnState = Arm_States.IN_MID_POS;
                break;
            case SCORE_LOW_GOAL:
                    returnState = Arm_States.IN_LOW_POS;
                    break;
            case IDLE:
            default:
                    returnState = Arm_States.IDLE;
                break;
        };
            return returnState;
    }

    //Helper Methods(mainly for logging and logic i guess)
    public Translation2d getDesiredPos(){
        return this.wantedMode.getDesiredPose();
    }
    
    public boolean areEqual(ArmSuperStructureState other){
        boolean areEqual = this.state == other.state;
        return areEqual;
    }
    

    //Methods used for the update state method
    public Translation2d getCurrentArmPos(){
        return currentArmPos;
    }

    public void setArmCurrentPos(Translation2d currentArmPose){
        this.currentArmPos = currentArmPose;
    }
    
    public boolean isInRange(){
        return this.currentArmPos == wantedMode.getDesiredPose();
    }

    public void updateState(){
        isInRange = isWantedState ? true : isInRange();

        switch(wantedMode){
            case PICK_ITEMS_POS:
                    state = isInRange? Arm_States.IN_PICK_POS : Arm_States.MOVING_TO_PICK ;
                break;
            case SCORE_HIGH_GOAL:
                    state = isInRange? Arm_States.IN_HIGH_POS : Arm_States.MOVING_TO_HIGH ;
                break;
            case SCORE_MID_GOAL:
                    state = isInRange? Arm_States.IN_MID_POS : Arm_States.MOVING_TO_MID ;
                break;
            case SCORE_LOW_GOAL:
                    state = isInRange? Arm_States.IN_LOW_POS : Arm_States.MOVING_TO_LOW ;
                    break;
            case IDLE:
            default:
                    state = Arm_States.IDLE;
                break;
            }
    }
    
}
