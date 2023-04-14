// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.ElementUnits;
import frc.robot.subsystems.DriveTrain;

public class DriveToPosition extends CommandBase {
  private PIDController positionPID;

  private final DriveTrain m_drive;
  private final double m_position;

  private static final double tolerance = ElementUnits.inchesToTicks(0.25);

  SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.DriveConstants.kS,
      Constants.DriveConstants.kV, Constants.DriveConstants.kA);
  
  public DriveToPosition(DriveTrain drive, double position) {

    this.m_drive = drive;
    this.m_position = position;

    positionPID = new PIDController(Constants.DriveConstants.kPositionP, Constants.DriveConstants.kPositionI, Constants.DriveConstants.kPositionD);

    positionPID.setTolerance(tolerance);
      
    addRequirements(this.m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveTrain.resetSensors();
    positionPID.setSetpoint(ElementUnits.inchesToTicks(this.m_position));
    // this.m_drive.setPower(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = positionPID.calculate(this.m_drive.getAverageRawEncoderTicks());
    double[] outputs = this.m_drive.getStraightOutput(power, power, 0);
   
    this.m_drive.setPower(MathUtil.clamp(outputs[0], -Constants.DriveConstants.kHoldPositionClamp, Constants.DriveConstants.kHoldPositionClamp), MathUtil.clamp(outputs[1], -Constants.DriveConstants.kHoldPositionClamp, Constants.DriveConstants.kHoldPositionClamp));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  // this.m_drive.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return positionPID.atSetpoint();
  }
}
