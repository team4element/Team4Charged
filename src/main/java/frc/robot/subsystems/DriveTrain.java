// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;
// import com.ctre.phoenix.music.Orchestra;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2Configuration;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.controllers.DriverController;

public class DriveTrain extends SubsystemBase {
  // Declare Motor Objects
  private static WPI_TalonFX leftFront = new WPI_TalonFX(Constants.DriveConstants.kLeftFrontMotor);
  private static WPI_TalonFX leftBack = new WPI_TalonFX(Constants.DriveConstants.kLeftBackMotor);

  private static WPI_TalonFX rightFront = new WPI_TalonFX(Constants.DriveConstants.kRightFrontMotor);
  private static WPI_TalonFX rightBack = new WPI_TalonFX(Constants.DriveConstants.kRightBackMotor);

  private static Pigeon2 pigeon = new Pigeon2(Constants.DriveConstants.kPigeon);

  // private static Gyro navX = new AHRS(SPI.Port.kMXP);

  // private final static MotorControllerGroup leftMotorControllerGroup = new MotorControllerGroup(leftFront, leftBack);
  // private final static MotorControllerGroup rightMotorControllerGroup = new MotorControllerGroup(rightFront, rightBack);

  private final static DifferentialDrive differentialDrive = new DifferentialDrive(leftBack, rightBack);
  // private final static DifferentialDrive differentialDrive = new DifferentialDrive(leftMotorControllerGroup, rightMotorControllerGroup);

  private final DifferentialDriveOdometry m_odometry;

  private Field2d m_field = new Field2d();
  
  public static DriverController mDriverController = new DriverController();

  private static double gearRatio = 9.06;

  private static double ticksPerInch = 2048 * gearRatio / 18.85;

  SlewRateLimiter filter = new SlewRateLimiter(1.7);

  public Orchestra sound;

  public DriveTrain() {
    // sound = new Orchestra();
    // sound.addInstrument(leftBack);
    // sound.addInstrument(leftFront);
    // sound.addInstrument(rightBack);
    // sound.addInstrument(rightFront);
    // sound.loadMusic("music/skril.chrp");
    // Current limit to prevent browning out due to too much current drawn 
    var stator = new StatorCurrentLimitConfiguration(true, 80, 100, 0.05);
    var supply = new SupplyCurrentLimitConfiguration(true, 40, 50, 0.05);
    
    leftBack.configStatorCurrentLimit(stator);
    leftBack.configSupplyCurrentLimit(supply);
    rightBack.configStatorCurrentLimit(stator);
    rightBack.configSupplyCurrentLimit(supply);


    // Make motors Follow the Leader
    leftFront.follow(leftBack);
    rightFront.follow(rightBack);

    leftFront.setInverted(false);
    leftBack.setInverted(false);

    rightFront.setInverted(true);
    rightBack.setInverted(true);

    // rightMotorControllerGroup.setInverted(true);
    // leftMotorControllerGroup.setInverted(true);

    leftBack.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightBack.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    leftBack.configOpenloopRamp(0.5);
    rightBack.configOpenloopRamp(0.5);

    leftBack.configClosedloopRamp(0.5);
    rightBack.configClosedloopRamp(0.5);
    
    configurePIDF();
    
    Pigeon2Configuration config = new Pigeon2Configuration();
    config.MountPosePitch = 90;
    config.MountPoseRoll = 180;
    pigeon.configAllSettings(config);
    zeroGyro();

    // navX.reset();
    // navX.calibrate();
    resetEncoders();

    m_odometry = new DifferentialDriveOdometry(getRotation2d(), 0, 0);
    m_odometry.resetPosition(getRotation2d(), 0, 0, new Pose2d());
    // m_odometry = new DifferentialDriveOdometry(navX.getRotation2d(), 0, 0);
    // m_odometry.resetPosition(navX.getRotation2d(), 0, 0, new Pose2d());
  }

  private void configurePIDF() {
    leftBack.config_kP(0, 0.006);
    leftBack.config_kI(0, 0.0);
    leftBack.config_kD(0, 0.0);
    leftBack.config_kF(0, 0.0);

    rightBack.config_kP(0, 0.006);
    rightBack.config_kI(0, 0.0);
    rightBack.config_kD(0, 0.0);
    rightBack.config_kF(0, 0.0);

    leftBack.config_kP(1, 0.0001);
    leftBack.config_kI(1, 0.0);
    leftBack.config_kD(1, 0.0);
    leftBack.config_kF(1, 0.0);

    rightBack.config_kP(1, 0.0001);
    rightBack.config_kI(1, 0.0);
    rightBack.config_kD(1, 0.0);
    rightBack.config_kF(1, 0.0);
  }

  public static void resetEncoders() {
    leftBack.setSelectedSensorPosition(0);
    leftFront.setSelectedSensorPosition(0);
    rightBack.setSelectedSensorPosition(0);
    rightFront.setSelectedSensorPosition(0);
  }

  public static void zeroGyro() {
    pigeon.setYaw(0);
  }

  public static Rotation2d getRotation2d() {
    return (Constants.DriveConstants.invertedGyro) ? Rotation2d.fromDegrees(360 - pigeon.getYaw()) : Rotation2d.fromDegrees(pigeon.getYaw());
}
  // public Gyro getGyro() {
  //   return navX;
  // }

  public Field2d getField() {
    return m_field;
  }
  
  public DifferentialDriveOdometry getOdometry() {
    return m_odometry;
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    // m_odometry.resetPosition(navX.getRotation2d(), 0, 0, pose);
    m_odometry.resetPosition(getRotation2d(), 0, 0, pose);
  }

  public double getRightEncoderPosition() {
    return rightBack.getSelectedSensorPosition() * Constants.DriveConstants.kLinearDistancePerMotorRotation;
  }

  public double getLeftEncoderPosition() {
    return leftBack.getSelectedSensorPosition() * Constants.DriveConstants.kLinearDistancePerMotorRotation;
  }

  public double getRightEncoderVelocity() {
    return rightBack.getSelectedSensorVelocity() * (Constants.DriveConstants.kLinearDistancePerMotorRotation / 60);
  }

  public double getLeftEncoderVelocity() {
    return leftBack.getSelectedSensorVelocity() * (Constants.DriveConstants.kLinearDistancePerMotorRotation / 60);
  }

  public DifferentialDrive getDifferentialDrive() {
    return differentialDrive;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }
  
  public void arcadeDrive(double fwd, double rot){
    // sound.play();
    differentialDrive.arcadeDrive(fwd, rot);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    // leftMotorControllerGroup.setVoltage(leftVolts);
    // rightMotorControllerGroup.setVoltage(rightVolts);
    leftBack.setVoltage(leftVolts);
    rightBack.setVoltage(rightVolts);

    differentialDrive.feed();
  }

  public double getAverageEncoderDistance() {
    return (getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0;
  }

  public static void zeroHeading() {
    // navX.calibrate();
    // navX.reset();
    zeroGyro();
    resetEncoders();
  }

  public static double getHeading() {
    // return navX.getRotation2d().getDegrees();
    return getRotation2d().getDegrees();
  }

  // public double getTurnRate() {
  //   return navX.getRate();
  // }

  public void setMaxOutput(double maxOutput) {
    differentialDrive.setMaxOutput(maxOutput);
  }

  public double calculateSlew(double input) {
    return filter.calculate(input);
  }

  public void setBrakeMode() {
    leftBack.setNeutralMode(NeutralMode.Brake);
    leftFront.setNeutralMode(NeutralMode.Brake);
    rightBack.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);
    SmartDashboard.putBoolean("Brake Mode", true);
  }

  public void setCoastMode() {
    leftBack.setNeutralMode(NeutralMode.Coast);
    leftFront.setNeutralMode(NeutralMode.Coast);
    rightBack.setNeutralMode(NeutralMode.Coast);
    rightFront.setNeutralMode(NeutralMode.Coast);
    SmartDashboard.putBoolean("Coast Mode", false);
  }
  
  public boolean slowTurnLeft() {
    return mDriverController.getSlowTurnLeft();
  }

  public boolean slowTurnRight() {
    return mDriverController.getSlowTurnRight();
  }
  
  public void setLeftTargetPosition(double position) {
    leftBack.set(TalonFXControlMode.Position, position * ticksPerInch);
  }

  public void setRightTargetPosition(double position) {
    rightBack.set(TalonFXControlMode.Position, position * ticksPerInch);
  }

  public double getLeftEncoderDistance() {
    return leftBack.getSelectedSensorPosition(0) / ticksPerInch;
  }

  public double getRightEncoderDistance() {
    return rightBack.getSelectedSensorPosition(0) / ticksPerInch;
  }

  public double getAverageRawEncoderTicks(){
    return ((rightBack.getSelectedSensorPosition(0)) + leftBack.getSelectedSensorPosition(0))/2;
  }

  public double getRightRawEncoderTicks() {
    return rightBack.getSelectedSensorPosition();
  }

  public double[] getStraightOutput(double l, double r, double target) {
    final double angleTolerance = .5;
    double l_out = l;
    double r_out = r;
    // double currentAngle = target- this.getGyroAngle();
    double currentAngle = target - getHeading();

    if (Math.abs(currentAngle) > angleTolerance) {
      double modifier = currentAngle * Constants.DriveConstants.kAngleP;
      // l_out += modifier;
      // r_out -= modifier;
      l_out -= modifier;
      r_out += modifier;
    }

    return new double[] {l_out, r_out};
  }

  public void setPower(double leftPower, double rightPower) {
    SmartDashboard.putNumber("left power", leftPower);
    SmartDashboard.putNumber("right power", rightPower);
    leftBack.set(TalonFXControlMode.PercentOutput, leftPower);
    rightBack.set(TalonFXControlMode.PercentOutput, rightPower);
  }

  public void setPosition(double position) {
    leftBack.set(TalonFXControlMode.Position, position);
    rightBack.set(TalonFXControlMode.Position, position);
  }

  public static void resetSensors() {
    // navX.reset();
    zeroGyro();
    resetEncoders();
  }

  public double getGyroAngle() {
    return pigeon.getYaw();
  }

  public double getPigeonPitch() {
    return pigeon.getPitch();
  }

  public boolean rotate() {
    return mDriverController.getRotate();
  }

  public boolean balance() {
    return mDriverController.getBalance();
  }

  @Override
  public void periodic() {
    // sound.play();

    SmartDashboard.putNumber("heading", pigeon.getYaw());
    SmartDashboard.putNumber("pitch", pigeon.getPitch());

    SmartDashboard.putNumber("leftEncoder distance", getLeftEncoderPosition());
    SmartDashboard.putNumber("rightEncoder distance", getRightEncoderPosition());

    SmartDashboard.putNumber("leftEncoder velocity", getLeftEncoderVelocity());
    SmartDashboard.putNumber("rightEncoder velocity", getRightEncoderVelocity());

    SmartDashboard.putNumber("leftEncoder raw", leftBack.getSelectedSensorPosition());
    SmartDashboard.putNumber("rightEncoder raw", rightBack.getSelectedSensorPosition());

    m_field.setRobotPose(m_odometry.getPoseMeters());

    m_odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftEncoderPosition(),
      getRightEncoderPosition());

    // m_odometry.update(navX.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition());
    m_odometry.update(getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition());
  }
}