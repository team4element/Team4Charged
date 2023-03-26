// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;

public class RaiseArmAndPivot extends ParallelCommandGroup {
  /** Creates a new RaiseArmAndPivot. */
  private final double m_angle;
  private final Arm m_arm;
  
  public RaiseArmAndPivot(Arm arm, double angle) {
    this.m_angle = angle;
    this.m_arm = arm;

    addCommands(new HoldArmPosition(this.m_arm, this.m_angle), new TogglePivot(this.m_arm));
  }
}
