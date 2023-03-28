// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double TalonFXEncoderPPR = 2048.0;

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

    public static final double kDriveGearRatio = 9.06;
    public static final double kDriveWheelRadiusInches = 3;

    public static final double kLinearDistancePerMotorRotation = Units.inchesToMeters((2* Math.PI * kDriveWheelRadiusInches) / (2048 * kDriveGearRatio));

    public static final double kTrackWidthMeters = Units.inchesToMeters(29.5);
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
    
    public static final double kMaxSpeedMetersPerSecond = 3.0;
    public static final double kMaxAccelerationPerSecondSquared = 3.0;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    // RotateToAngle PID
    public static final double kAngleP = 0.009;
    public static final double kAngleI = 0;
    public static final double kAngleD = 0;
    public static final double kAngleF = 0;

    // DriveStraight PID
    public static final double kDriveP = 0.003;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;
    public static final double kDriveF = 0;
    
    // HoldDrivePosition PID
    public static final double kPositionP = 0.000025;
    public static final double kPositionI = 0;
    public static final double kPositionD = 0;
    public static final double kPositionF = 0;

    // Feedforward Constants
    public static final double kS = 0.12202;
    public static final double kV = 2.054;
    public static final double kA = 0.5882;
    public static final double kPDriveVel = 1.1695;
  }

  public static class PneumaticsConstants {
    //Pneumatics
    public static final int kCompressorID = 0;

    // Arm Solenoids
    public static final int kLeftArmSolenoid = 2;
    public static final int kRightArmSolenoid = 1;

    // Intake Solenoid
    public static final int kIntakeSolenoid = 0;
  }

  public static class ArmConstants {
    // Arm
    public static final int kLeftMotor = 5;
    public static final int kRightMotor = 6;

    public static final double kGearRatio = .004;

    // Store setpoint values in degrees
    // todo find exact values
     public static final int kMidSetpoint = 90;
     public static final int kHighSetpoint = 0;

    // Arm Position PID
    public static final double kArmP = .0001;
    public static final double kArmI = 0;
    public static final double kArmD = 0.001;
    public static final double kArmF = 0;
    
    // Feedforward Constants
    public static final double kS = 0.041549;
    public static final double kG = 0.011533;
    public static final double kV = 4.3067;
    public static final double kA = 0.097108;

    // Default Angles
    public static final double kBaseAngle = .75;
    public static final double kPivotedAngle = 47;
  }
  
  public static class IntakeConstants {
    // Intake
    public static final int kLeftMotor = 7;
    public static final int kRightMotor = 8;

    // Teleop Power
    public static final double kTeleopIntakeForwardPower = .5;
    public static final double kTeleopIntakeReversePower = -.3;

    // Auto Power
    public static final double kAutoIntakeReversePower = -1;
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
