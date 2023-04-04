// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Arm;

public class HighConePosition extends SequentialCommandGroup {
  private final Arm m_arm;

  public HighConePosition(Arm arm) {
    this.m_arm = arm;

    addCommands(new TogglePivot(this.m_arm), new HoldArmPosition(this.m_arm, 112));
  }
}
