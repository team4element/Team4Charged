package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

public class ScoreLowAndBalance extends SequentialCommandGroup {
  private final DriveTrain m_drive;
  private final Intake m_intake;

  public ScoreLowAndBalance(DriveTrain drive, Intake intake) {
    this.m_drive = drive;
    this.m_intake = intake;

    addCommands(new Score(this.m_intake).withTimeout(.25),
        new ProfiledDriveToPosition(this.m_drive, -148).withTimeout(4),
        new ProfiledDriveToPosition(this.m_drive, 68),
        new Balance(this.m_drive));
  }
}
