// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.RobotContainer;

public class FollowTrajectory extends SequentialCommandGroup {
  public Command loadPathWeaverTrajectoryCommand(String filename, boolean resetOdometry) {

    Trajectory trajectory;
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + filename, ex.getStackTrace());
      System.out.println("Unable to read from file " + filename);
      return new InstantCommand();
    }

    Command ramseteCommand = new RamseteCommand(
      trajectory,
      RobotContainer.getDriveTrainSubsystem()::getPose,
      new RamseteController(Constants.DriveConstants.kRamseteB, Constants.DriveConstants.kRamseteZeta),
      new SimpleMotorFeedforward(
        DriveConstants.kS,
        DriveConstants.kV,
        DriveConstants.kA),
      DriveConstants.kDriveKinematics,
      RobotContainer.getDriveTrainSubsystem()::getWheelSpeeds,
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      new PIDController(DriveConstants.kPDriveVel, 0, 0),

      RobotContainer.getDriveTrainSubsystem()::tankDriveVolts,
      RobotContainer.getDriveTrainSubsystem());

    if (resetOdometry) {
      return new SequentialCommandGroup(
        new InstantCommand(() -> RobotContainer.getDriveTrainSubsystem()
          .resetOdometry(trajectory.getInitialPose())),
        ramseteCommand);
    } else {
      return ramseteCommand;
    }
    
  }
  
  public FollowTrajectory(DriveTrain driveTrain) {
    DriveTrain.resetEncoders();
    addCommands(loadPathWeaverTrajectoryCommand("pathplanner/generatedJSON/test.wpilib.json", true));

    addRequirements(driveTrain);
  }

}
 