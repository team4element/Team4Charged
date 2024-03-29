package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class ScoreMidAndIntake extends SequentialCommandGroup {
  private final DriveTrain m_drive;
  private final Arm m_arm;
  private final Intake m_intake;

  public ScoreMidAndIntake(DriveTrain drive, Arm arm, Intake intake) {
    this.m_drive = drive;
    this.m_arm = arm;
    this.m_intake = intake;

    addCommands(new HoldArmPosition(this.m_arm, 74.5).withTimeout(1.4),
    new ProfiledDriveToPosition(this.m_drive, 11.5),
    new ToggleClaw(this.m_intake), 
    new LowerArmAndDrive(this.m_drive, this.m_arm, -95).withTimeout(3),
    new RotateToAngle(this.m_drive, 180).withTimeout(2),
    new ProfiledDriveToPosition(this.m_drive, 95, 2),
    new LowerArmDown(this.m_arm).withTimeout(1),
    new IntakeForward(this.m_intake).withTimeout(3),
    new RotateToAngle(this.m_drive, 180).withTimeout(2));
  }
}
