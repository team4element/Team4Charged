// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Arm;

public class ToggleCompressor extends CommandBase {
  private final Arm m_arm;

  public ToggleCompressor(Arm arm) {
    this.m_arm = arm;
    addRequirements(this.m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_arm.mCompressor.isEnabled()) {
      this.m_arm.mCompressor.disable();
    } else {
      this.m_arm.mCompressor.enableDigital();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
