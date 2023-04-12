package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

public class ScoreLowAndIntake extends SequentialCommandGroup {
  private final DriveTrain m_drive;
  private final Intake m_intake;

  public ScoreLowAndIntake(DriveTrain drive, Intake intake) {
    this.m_drive = drive;
    this.m_intake = intake;

    addCommands(new Score(this.m_intake).withTimeout(1),
    new DriveToPosition(this.m_drive, -95).withTimeout(4),
    new RotateToAngle(this.m_drive, 180).withTimeout(0.5),
    new ToggleClaw(this.m_intake),
    new DriveToPosition(this.m_drive, 95).withTimeout(3),
    new IntakeForward(this.m_intake).withTimeout(1),
    new RotateToAngle(this.m_drive, 180).withTimeout(0.5));
  }
}
