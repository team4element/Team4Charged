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
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;
import frc.robot.controllers.DriverController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  // Declare Motor Objects
  private WPI_TalonFX leftFront;
  private WPI_TalonFX leftBack;

  private WPI_TalonFX rightFront;
  private WPI_TalonFX rightBack;

  private AHRS navX;

  public static DriverController mDriverController = new DriverController();

  private static double gearRatio = 9.06;

  private static double ticksPerInch = 2048 * gearRatio / 18.85;

  SlewRateLimiter filter = new SlewRateLimiter(.9);

  public DriveTrain() {
    // Define Motor Objects
    leftFront = new WPI_TalonFX(Constants.DriveConstants.kLeftFrontMotor);
    leftBack = new WPI_TalonFX(Constants.DriveConstants.kLeftBackMotor);

    leftBack.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    rightFront = new WPI_TalonFX(Constants.DriveConstants.kRightFrontMotor);
    rightBack = new WPI_TalonFX(Constants.DriveConstants.kRightBackMotor);
   
    // Current limit to prevent browning out due to too much current drawn 
    var stator = new StatorCurrentLimitConfiguration(true, 80, 100, 0.05);
    var supply = new SupplyCurrentLimitConfiguration(true, 40, 50, 0.05);

    leftBack.configStatorCurrentLimit(stator);
    leftBack.configSupplyCurrentLimit(supply);
    rightBack.configStatorCurrentLimit(stator);
    rightBack.configSupplyCurrentLimit(supply);

    navX = new AHRS(SPI.Port.kMXP);

    // Make motors Follow the Leader
    leftFront.follow(leftBack);

    rightFront.follow(rightBack);

    leftBack.setInverted(true);
    leftFront.setInverted(true);

    leftBack.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightBack.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    leftBack.configOpenloopRamp(0.5);
    rightBack.configOpenloopRamp(0.5);

    leftBack.configClosedloopRamp(0.5);
    rightBack.configClosedloopRamp(0.5);
    
    configurePIDF();

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

  public double getAverageEncoderDistance() {
    return (getLeftEncoderDistance() + getRightEncoderDistance())/2;
  }

  public double[] getStraightOutput(double l, double r, double target) {
    final double angleTolerance = 1;
    double l_out = l;
    double r_out = r;
    double currentAngle = getGyro() - target;

    if (Math.abs(currentAngle) > angleTolerance) {
      double modifier = currentAngle * Constants.DriveConstants.kAngleP;
      l_out += modifier;
      r_out -= modifier;
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

  public void resetSensors() {
    navX.reset();
    leftBack.setSelectedSensorPosition(0);
    rightBack.setSelectedSensorPosition(0);
  }

  public double getGyro() {
    return navX.getAngle();
  }

  public double getGyroPitch(){
    return navX.getPitch();
  }

  public double getGyroRoll(){
    return navX.getRoll();
  }

  public boolean rotate() {
    return mDriverController.getRotate();
  }

  @Override
  public void periodic() {
    // System.out.println(getGyro());
  }
}