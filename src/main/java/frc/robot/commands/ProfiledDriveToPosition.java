// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.util.Units;

import frc.robot.Constants;
import frc.robot.ElementUnits;
import frc.robot.subsystems.DriveTrain;

public class ProfiledDriveToPosition extends CommandBase {
  
  private ProfiledPIDController positionPID;
  // private PIDController positionPID;


  private final DriveTrain m_drive;
  private final double m_position;

  private static final double tolerance = ElementUnits.inchesToTicks(0.1);

  SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.DriveConstants.kS,
      Constants.DriveConstants.kV, Constants.DriveConstants.kA);
  
  public ProfiledDriveToPosition(DriveTrain drive, double position, double maxAcceleration) {

    this.m_drive = drive;
    this.m_position = position;
    //positionPID = new PIDController(Constants.DriveConstants.kProfiledPositionP, Constants.DriveConstants.kProfiledPositionI, Constants.DriveConstants.kProfiledPositionD);

    positionPID = new ProfiledPIDController(Constants.DriveConstants.kProfiledPositionP, Constants.DriveConstants.kProfiledPositionI, Constants.DriveConstants.kProfiledPositionD, new TrapezoidProfile.Constraints(3, maxAcceleration));
    positionPID.setTolerance(tolerance);
      
    addRequirements(this.m_drive);
  }

  public ProfiledDriveToPosition(DriveTrain drive, double position) {

    this.m_drive = drive;
    this.m_position = position;
    //positionPID = new PIDController(Constants.DriveConstants.kProfiledPositionP, Constants.DriveConstants.kProfiledPositionI, Constants.DriveConstants.kProfiledPositionD);

    positionPID = new ProfiledPIDController(Constants.DriveConstants.kProfiledPositionP, Constants.DriveConstants.kProfiledPositionI, Constants.DriveConstants.kProfiledPositionD, new TrapezoidProfile.Constraints(3, 1));
    positionPID.setTolerance(tolerance);
      
    addRequirements(this.m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveTrain.resetSensors();
    positionPID.setGoal(Units.inchesToMeters(this.m_position));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = positionPID.calculate(this.m_drive.getAverageEncoderDistance());
    double[] outputs = this.m_drive.getStraightOutput(power, power, 0);
    System.out.println(power);
    this.m_drive.setPower(outputs[0], outputs[1]);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return positionPID.atGoal();
  }
}
