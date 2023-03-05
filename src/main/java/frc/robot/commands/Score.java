package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Intake;
import frc.robot.Constants;

public class Score extends CommandBase {
  private final Intake m_intake;
  private final Timer m_timer = new Timer();

  public Score(Intake intake) {
    this.m_intake = intake;
    addRequirements(this.m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setIntakeReversePower(-Constants.IntakeConstants.kAutoIntakeReversePower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntakeReversePower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(1);
  }
}
