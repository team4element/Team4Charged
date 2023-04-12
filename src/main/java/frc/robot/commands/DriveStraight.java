package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants;
import frc.robot.ElementUnits;

public class DriveStraight extends CommandBase {
  PIDController drivePID;

  private final double m_distance;
  private final DriveTrain m_drive;

  private static final double tolerance = ElementUnits.inchesToTicks(2);
  // TODO: Set proper tolerance

  public DriveStraight(DriveTrain drive, double distance) {
    drivePID = new PIDController(Constants.DriveConstants.kDriveP, Constants.DriveConstants.kDriveI, Constants.DriveConstants.kDriveD);
    this.m_distance = distance;
    this.m_drive = drive;

    drivePID.setTolerance(tolerance);

    addRequirements(this.m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.m_drive.setCoastMode();
    DriveTrain.resetSensors();
    drivePID.setSetpoint(ElementUnits.inchesToTicks(this.m_distance));
    this.m_drive.setPower(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = drivePID.calculate(this.m_drive.getAverageEncoderDistance());
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
    return drivePID.atSetpoint();
  }
}
