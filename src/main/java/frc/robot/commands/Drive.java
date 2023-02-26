// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.controllers.DriverController;

public class Drive extends CommandBase {
  private final DriveTrain m_drive;
  DriverController mDriverController = new DriverController();

  public Drive(DriveTrain drive, DriverController controller) {
    this.m_drive = drive;
    this.mDriverController = controller;

    addRequirements(this.m_drive);
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double straightPower = this.mDriverController.getThrottle();
    double rotationPower = this.mDriverController.getTurn();
    m_drive.setPower(straightPower + rotationPower, straightPower - rotationPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.resetSensors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}


