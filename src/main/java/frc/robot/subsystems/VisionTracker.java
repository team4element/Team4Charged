// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.Constants;

public class VisionTracker extends SubsystemBase {

  private NetworkTable mCurrentTargetingLimelightNT;
  private NetworkTable limelightNT = NetworkTableInstance.getDefault().getTable("limelight");

  /** Creates a new VisionTracker. */
  private VisionTracker() {
    mCurrentTargetingLimelightNT = limelightNT;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double getX = limelightNT.getEntry("tx").getDouble(0);
    double getY = limelightNT.getEntry("ty").getDouble(0);
    double getArea = limelightNT.getEntry("ta").getDouble(0);
    double getDistance = (Constants.TargetingConstants.kTargetHeight - Constants.TargetingConstants.kFloorToLens) /
      Math.tan(Math.toRadians(Constants.TargetingConstants.kFloorToLensAngle + getY));
  
  }
}
