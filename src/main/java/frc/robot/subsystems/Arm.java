// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.ElementUnits;
import frc.robot.controllers.OperatorController;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  // Declaring Motors
  private WPI_TalonFX left;

  private WPI_TalonFX right;

  // Declaring Solenoids
  private static Solenoid mLeftPivotPiston;
  private static Solenoid mRightPivotPiston;

  // Declaring Controller
  public static OperatorController mOperatorController = new OperatorController();

  // Declaring Compressor
  public Compressor mCompressor;

  public double currentAngle;

  ArmFeedforward feedforward = new ArmFeedforward(Constants.ArmConstants.kS, Constants.ArmConstants.kG,
      Constants.ArmConstants.kV, Constants.ArmConstants.kA);

  public Arm() {
    // Instantiating Motors
    left = new WPI_TalonFX(Constants.ArmConstants.kLeftMotor);

    right = new WPI_TalonFX(Constants.ArmConstants.kRightMotor);
    
    left.setInverted(false);
    right.setInverted(true);

    right.follow(left);

    // Instantiating Solenoids
    mLeftPivotPiston = new Solenoid(0, PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.kLeftArmSolenoid);
    mRightPivotPiston = new Solenoid(0, PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.kRightArmSolenoid);

    // Instantiating Compressor
    mCompressor = new Compressor(Constants.PneumaticsConstants.kCompressorID, PneumaticsModuleType.CTREPCM);

    left.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    right.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    left.configReverseSoftLimitEnable(true, 0);

    left.configForwardSoftLimitThreshold(armAngleToTicks(98), 0);
    left.configReverseSoftLimitThreshold(armAngleToTicks(2), 0);

    left.configForwardSoftLimitEnable(true, 0);
    left.configReverseSoftLimitEnable(true, 0);
    
    var stator = new StatorCurrentLimitConfiguration(true, 80, 100, 0.05);
    var supply = new SupplyCurrentLimitConfiguration(true, 40, 50, 0.05);

    left.configStatorCurrentLimit(stator);
    left.configSupplyCurrentLimit(supply);

    right.configStatorCurrentLimit(stator);
    right.configSupplyCurrentLimit(supply);

    // left.config_kP(0,  Constants.ArmConstants.kDistanceP);
    // left.config_kI(0, Constants.ArmConstants.kDistanceI);
    // left.config_kD(0, Constants.ArmConstants.kDistanceD);

    // right.config_kP(0,  Constants.ArmConstants.kDistanceP);
    // right.config_kI(0, Constants.ArmConstants.kDistanceI);
    // right.config_kD(0, Constants.ArmConstants.kDistanceD);
    // left.config_kF(0, feedforward.calculate(Constants.ArmConstants.kMidSetpoint, Constants.ArmConstants.kVelocity,
    //    Constants.ArmConstants.kAcceleration));

  }

  public double armAngleToTicks(double angle) {
    return ElementUnits.rotationsToTicks((angle / 360) / Constants.ArmConstants.kGearRatio, Constants.TalonFXEncoderPPR); 
  }

  public void setBrakeMode() {
    left.setNeutralMode(NeutralMode.Brake);
    right.setNeutralMode(NeutralMode.Brake);
  }

  public void setCoastMode() {
    left.setNeutralMode(NeutralMode.Coast);
    right.setNeutralMode(NeutralMode.Coast);
  }

  public boolean toggleCompressor() {
    return mOperatorController.getCompressorToggle();
  }

  public boolean getTogglePivot() {
    return mOperatorController.getPivot();
  }

  public void getMotorOutput() {
    System.out.println(left.getMotorOutputPercent());
  }

  public double getEncoderDistance() {
    return left.getSelectedSensorPosition(0);
  }

  public void setArmPower(double power) {
    SmartDashboard.putNumber("arm input", power);
    // double filteredValue = filterForSafeValues(power);
    left.set(TalonFXControlMode.PercentOutput, power * 0.5);
  }

  public void setArmPosition(double position) {
    left.set(TalonFXControlMode.Position, position);
  }

  public void getYAxis() {
    System.out.println(mOperatorController.getThrottle());
  }

  public void resetSensors() {
    left.setSelectedSensorPosition(0);
  }

  public boolean getMidPosition() {
    return mOperatorController.midPosition();
  }

  public boolean getHighPosition() {
    return mOperatorController.highPosition();
  }

  // private boolean maxLimit() {
  // return (getArmAngle() >= Constants.ArmConstants.kMaxLimit);
  // }

  // private boolean minLimit() {
  // return (getArmAngle() <= Constants.ArmConstants.kMinLimit);
  // }

  // private double filterForSafeValues(double power){
  // if (maxLimit() && power < 0){
  // return power;
  // } else if (minLimit() && power > 0) {
  // return power;
  // } else if (!minLimit() && !maxLimit()){
  // return power;
  // } else {
  // return 0;
  // }
  // }

  // private boolean defaultPosition() {
  // return (!mLeftPivotPiston.get() && !mRightPivotPiston.get());
  // }

  // private boolean extendedPosition() {
  // return (mLeftPivotPiston.get() && mRightPivotPiston.get());
  // }

  public boolean getPivotState(){
    return mLeftPivotPiston.get();
  }

  public void togglePivot() {
    System.out.println("toggle pivot");
    mLeftPivotPiston.toggle();
    mRightPivotPiston.toggle();
  }

  public double getArmAngle() {
    currentAngle = ElementUnits.ticksToRotations(left.getSelectedSensorPosition(0), Constants.TalonFXEncoderPPR);
    currentAngle *= Constants.ArmConstants.kGearRatio; // gets rotation of the arm from rotations of the motors
    currentAngle *= 360; // gets the current angle of the arm in degrees
    // System.out.println(currentAngle);
    return currentAngle;
  }

  @Override
  public void periodic() {
    // getArmAngle();
    // System.out.println(getEncoderDistance());
  }
}
