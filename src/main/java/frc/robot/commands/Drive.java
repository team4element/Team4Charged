// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.controllers.DriverController;
import frc.robot.ElementMath;

public class Drive extends CommandBase {
  private final DriveTrain m_drive;
  DriverController mDriverController;

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
    // double straightPower = ElementMath.squareInput(ElementMath.handleDeadband(this.mDriverController.getThrottle(), Constants.ControllerConstants.kJoystickThreshold));
    double straightPower = ElementMath.squareInput(this.mDriverController.getThrottle());
    double rotationPower = -this.mDriverController.getTurn() * 0.5;

    if (this.mDriverController.getSlowDrive()){
      m_drive.setBrakeMode(); 
    } else {
      m_drive.setCoastMode();
    }
    
    // double rotationPower = ElementMath.cubeInput(ElementMath.handleDeadband(this.mDriverController.getTurn(), Constants.ControllerConstants.kJoystickThreshold));
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


