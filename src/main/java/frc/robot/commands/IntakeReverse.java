package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Intake;
import frc.robot.Constants;

public class IntakeReverse extends CommandBase {
  private final Intake m_intake;

  public IntakeReverse(Intake intake) {
    this.m_intake = intake;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setIntakeReversePower(Constants.IntakeConstants.kIntakeReversePower);
    /* If this doesn't work out, try running setIntakeReversePower function in initialize
    and run with .onTrue in RobotContainer */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntakeReversePower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
