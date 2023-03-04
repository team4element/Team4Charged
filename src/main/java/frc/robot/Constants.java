// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ControllerConstants {
    //Controllers
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kJoystickThreshold = 0.1;
  }
  
  public static class DriveConstants {
    //DriveTrain
    public static final int kLeftFrontMotor = 1;
    public static final int kLeftBackMotor = 2;

    public static final int kRightFrontMotor = 3;
    public static final int kRightBackMotor = 4;

    // RotateToAngle PID
    public static final double kAngleP = 0.006;
    public static final double kAngleI = 0;
    public static final double kAngleD = 0;
    public static final double kAngleF = 0;
  }

  public static class PneumaticsConstants {
    //Pneumatics
    public static final int kCompressorID = 0;

    // Arm Solenoids
    public static final int kLeftArmSolenoid = 0;
    public static final int kRightArmSolenoid = 1;

    // Intake Solenoid
    public static final int kIntakeSolenoid = 2;
  }

  public static class ArmConstants {
    // Arm
    public static final int kLeftMotor = 5;
    public static final int kRightMotor = 6;

    // Store setpoint values 
    // public static final int kMidSetpoint = 0;
    // public static final int kHighSetpoint = 72450;

    // Arm Mid/High Position PID
    public static final double kDistanceP = 0.00001;
    public static final double kDistanceI = 0;
    public static final double kDistanceD = 0;
    public static final double kDistanceF = 0;
    
    // Limit values not final
    public static final double kMaxLimit = 60;
    public static final double kMinLimit = 5;
  }
  
  public static class IntakeConstants {
    // Intake
    public static final int kLeftMotor = 7;
    public static final int kRightMotor = 8;

    public static final double kIntakeForwardPower = .5;
    public static final double kIntakeReversePower = -.5;
  }

  public static class TargetingConstants {
    //Limelight
    public static final double kHorizontalFOV = 59.6; // degrees
    public static final double kVerticalFOV = 49.7; // degrees
    public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
    public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));
    public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds

    public static final double kFloorToLens = 22.75; // To be modified
    public static final double kFloorToLensAngle = 15.233; //  To be modified
    public static final double kFloorToTarget = 83 + (30/4); // To be modified
    public static final double kTargetHeight = 20; // To be modified
    /* 
    public static final double kMaxTrackerDistance;
    public static final double kMaxGoalTrackAge;
    public static final double kMaxGoalTrackAgeNotTracking;
    public static final double kMaxGoalTrackSmoothingTime;
    public static final double kTrackStabilityWeight;
    public static final double kTrackAgeWeight;
    public static final double kTrackSwitchingWeight;
   */
  
    public static final double kCameraFrameRate = 90.0;
    public static final double kMinStability = 0.5;
  }
}
