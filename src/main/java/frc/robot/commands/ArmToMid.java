package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Arm;
import frc.robot.Constants;

public class ArmToMid extends CommandBase {
  PIDController distancePID;

  private final double m_distance;
  private final Arm m_arm;

  private static final double tolerance = 100;

  public ArmToMid(Arm arm, double distance) {
    distancePID = new PIDController(Constants.ArmConstants.kDistanceP, Constants.ArmConstants.kDistanceI, Constants.ArmConstants.kDistanceD);
    this.m_distance = distance;
    this.m_arm = arm;

    distancePID.setTolerance(tolerance);

    addRequirements(this.m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distancePID.setSetpoint(this.m_distance);
    this.m_arm.setArmPower(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pidPower = distancePID.calculate(this.m_arm.getArmAngle());
    this.m_arm.setArmPower(pidPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_arm.setArmPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
