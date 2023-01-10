// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;

public class Drive extends CommandBase {
  private final DriveTrain m_drive;
  private final CommandXboxController controller;
  
  public Drive(DriveTrain drive, CommandXboxController controller) {
    this.m_drive = drive;
    this.controller = controller;

    addRequirements(this.m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftPower = this.controller.getLeftY();
    double rightPower = this.controller.getRightY();
    m_drive.setPower(leftPower, rightPower);
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
