// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {

  public static CommandBase taxiAuto(DriveTrain drive) {
    return Commands.sequence(new TaxiAuto(RobotContainer.m_driveTrain));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
