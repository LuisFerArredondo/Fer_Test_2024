// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SuperStructures.Arm.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructures.Arm.ArmSuperStructure;
import frc.robot.SuperStructures.Arm.ArmSuperStructureState;
import frc.robot.SuperStructures.Arm.ArmSuperStructureState.Arm_Modes;

public class ArmCommand extends Command {
  private ArmSuperStructure armSuperStructure;
  private Arm_Modes desiredMode;

  /** Creates a new LiftHigh. */
  public ArmCommand(ArmSuperStructure armSuperStructure, ArmSuperStructureState.Arm_Modes desiredMode) {
    this.armSuperStructure = armSuperStructure;
    this.desiredMode = desiredMode;

    addRequirements(armSuperStructure);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSuperStructure.setMode(desiredMode);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSuperStructure.allowArmMovement();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
