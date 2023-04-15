package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class ScoreHighAndIntake extends SequentialCommandGroup {
  private final DriveTrain m_drive;
  private final Arm m_arm;
  private final Intake m_intake;

  public ScoreHighAndIntake(DriveTrain drive, Arm arm, Intake intake) {
    this.m_drive = drive;
    this.m_arm = arm;
    this.m_intake = intake;

    addCommands(new HighConePosition(this.m_arm).withTimeout(2.5),
    new ProfiledDriveToPosition(this.m_drive, 23.5),
    new ToggleClaw(this.m_intake), 
    new ProfiledDriveToPosition(this.m_drive, -24),
    new TogglePivot(this.m_arm),
    new LowerArmAndDrive(this.m_drive, this.m_arm, -71).withTimeout(3),
    new RotateToAngle(this.m_drive, 180).withTimeout(0.5),
    new ProfiledDriveToPosition(this.m_drive, 95, 2),
    new IntakeForward(this.m_intake).withTimeout(1),
    new RotateToAngle(this.m_drive, 180).withTimeout(0.5));
  }
}
