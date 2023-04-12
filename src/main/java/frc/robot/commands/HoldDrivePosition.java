// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.ElementUnits;
import frc.robot.subsystems.DriveTrain;

public class HoldDrivePosition extends CommandBase {

  private PIDController positionPID;

  private final DriveTrain m_drive;

  private static final double tolerance = ElementUnits.inchesToTicks(0.25);

  SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.DriveConstants.kS,
      Constants.DriveConstants.kV, Constants.DriveConstants.kA);
  
  public HoldDrivePosition(DriveTrain drive) {

    this.m_drive = drive;

    positionPID = new PIDController(Constants.DriveConstants.kPositionP, Constants.DriveConstants.kPositionI, Constants.DriveConstants.kPositionD);

    positionPID.setTolerance(tolerance);
      
    addRequirements(this.m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveTrain.resetSensors();
    positionPID.setSetpoint(ElementUnits.inchesToTicks(0));
    // this.m_drive.setPower(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = positionPID.calculate(this.m_drive.getAverageRawEncoderTicks(), 0);
    double[] outputs = this.m_drive.getStraightOutput(power, power, 0);
    this.m_drive.setPower(outputs[0], outputs[1]);
  }

  // Called once the command ends or is interrupted.
  @Override
public void end(boolean interrupted) {
  this.m_drive.setPower(0, 0);
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
