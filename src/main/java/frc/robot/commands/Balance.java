package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class Balance extends CommandBase {
  private final DriveTrain m_drive;
  private PIDController balancePID;
  
  private final double tolerance = 5;

  private double power;

  public Balance(DriveTrain drive) {
    this.m_drive = drive;
    
    balancePID = new PIDController(Constants.DriveConstants.kBalanceP, Constants.DriveConstants.kBalanceI, Constants.DriveConstants.kBalanceD);

    balancePID.setTolerance(tolerance);

    addRequirements(this.m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    balancePID.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    power = -balancePID.calculate(this.m_drive.getPigeonPitch());
    this.m_drive.arcadeDrive((MathUtil.clamp(power, -Constants.DriveConstants.kBalanceClamp, Constants.DriveConstants.kBalanceClamp)), 0);


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
