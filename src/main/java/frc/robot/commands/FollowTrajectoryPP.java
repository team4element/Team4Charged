// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class FollowTrajectoryPP extends SequentialCommandGroup {
  public Command TaxiAndBalance(Boolean resetOdometry) {

    PathPlannerTrajectory trajectory = PathPlanner.loadPath("TaxiAndBalance", new PathConstraints(1.5, 1));

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("holdPosition", new HoldDrivePosition(RobotContainer.getDriveTrainSubsystem()));
    eventMap.put("intake", new Score(RobotContainer.m_intake));

    RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
        RobotContainer.getDriveTrainSubsystem()::getPose,
        RobotContainer.getDriveTrainSubsystem()::resetOdometry,
        new RamseteController(Constants.DriveConstants.kRamseteB, Constants.DriveConstants.kRamseteZeta),
        DriveConstants.kDriveKinematics,
        new SimpleMotorFeedforward(
            DriveConstants.kS,
            DriveConstants.kV,
            DriveConstants.kA),
        RobotContainer.getDriveTrainSubsystem()::getWheelSpeeds,
        new PIDConstants(DriveConstants.kPDriveVel, 0, 0),
        RobotContainer.getDriveTrainSubsystem()::tankDriveVolts,
        eventMap,
        true,
        RobotContainer.getDriveTrainSubsystem());

    Command taxiAndBalance = autoBuilder.fullAuto(trajectory);
    // Command taxiAndBalance = new PPRamseteCommand(
    //     trajectory,
    //     RobotContainer.getDriveTrainSubsystem()::getPose,
    //     new RamseteController(Constants.DriveConstants.kRamseteB, Constants.DriveConstants.kRamseteZeta),
    //     new SimpleMotorFeedforward(
    //         DriveConstants.kS,
    //         DriveConstants.kV,
    //         DriveConstants.kA),
    //     DriveConstants.kDriveKinematics,
    //     RobotContainer.getDriveTrainSubsystem()::getWheelSpeeds,
    //     new PIDController(DriveConstants.kPDriveVel, 0, 0),
    //     new PIDController(DriveConstants.kPDriveVel, 0, 0),
    //     RobotContainer.getDriveTrainSubsystem()::tankDriveVolts,
    //     true,
    //     RobotContainer.getDriveTrainSubsystem());

    if (resetOdometry) {

      return new SequentialCommandGroup(
          new InstantCommand(() -> RobotContainer.getDriveTrainSubsystem()
              .resetOdometry(trajectory.getInitialPose())),
          taxiAndBalance);
    }

    return taxiAndBalance;
  }

  public FollowTrajectoryPP(DriveTrain driveTrain) {
    // DriveTrain.resetEncoders();
    addCommands(TaxiAndBalance(false));

    addRequirements(driveTrain);
  }

}
