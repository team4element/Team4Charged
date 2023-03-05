package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TaxiAuto extends CommandBase {
  private final DriveTrain m_drive;

  private final static double TARGET_POSITION = -190;

  public TaxiAuto(DriveTrain drive) {
    this.m_drive = drive;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetSensors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.setLeftTargetPosition(TARGET_POSITION);
    m_drive.setRightTargetPosition(TARGET_POSITION);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setPower(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(TARGET_POSITION - m_drive.getLeftEncoderDistance()) < -1.0;
  }
}
