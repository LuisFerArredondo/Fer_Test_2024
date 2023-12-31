// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.StateMachineToolBox;

public interface StateBasedSubsystem {
    public void updateState(boolean isMoving);

    public void updateDirection();

    public Directions getDirection();

}
