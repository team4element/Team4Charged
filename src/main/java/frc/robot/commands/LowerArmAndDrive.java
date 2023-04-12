package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Arm;

public class LowerArmAndDrive extends ParallelCommandGroup {
  private final DriveTrain m_drive;
  private final Arm m_arm;

  public LowerArmAndDrive(DriveTrain drive, Arm arm, double position) {
    this.m_drive = drive;
    this.m_arm = arm;

    addCommands(new LowerArmDown(this.m_arm), new DriveToPosition(this.m_drive, position));
  }
}
