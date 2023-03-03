package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Intake;

public class ToggleClaw extends CommandBase {
  private final Intake m_intake;
  
  public ToggleClaw(Intake intake) {
    this.m_intake = intake;
    addRequirements(this.m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.m_intake.getToggleClaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
