// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.ElementMath;
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

  ArmFeedforward feedforward = new ArmFeedforward(Constants.ArmConstants.kS, Constants.ArmConstants.kG, Constants.ArmConstants.kV, Constants.ArmConstants.kA);
  
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

    // look into position limits

     left.config_kP(0, Constants.ArmConstants.kDistanceP);
     left.config_kI(0, Constants.ArmConstants.kDistanceI);
     left.config_kD(0, Constants.ArmConstants.kDistanceD);
     left.config_kF(0, feedforward.calculate(Constants.ArmConstants.kMidSetpoint, Constants.ArmConstants.kVelocity, Constants.ArmConstants.kAcceleration));
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

  // public double getEncoderDistance() {
  //   return left.getSelectedSensorPosition(0);
  // }

  public void setArmPower(double power) {
    SmartDashboard.putNumber("arm input", power);
    // double filteredValue = filterForSafeValues(power);
    left.set(TalonFXControlMode.PercentOutput, -power * 0.5);
  }

  public void setArmPosition(double position) {
    left.set(TalonFXControlMode.Position, position);
  }

  public void getYAxis(){
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
  //   return (getArmAngle() >= Constants.ArmConstants.kMaxLimit);
  // }

  // private boolean minLimit() {
  //   return (getArmAngle() <= Constants.ArmConstants.kMinLimit);
  // }

  // private double filterForSafeValues(double power){
  //   if (maxLimit() && power < 0){
  //     return power;
  //   } else if (minLimit() && power > 0) {
  //     return power;
  //   } else if (!minLimit() && !maxLimit()){
  //     return power;
  //   } else {
  //     return 0;
  //   }
  // }
    
  // private boolean defaultPosition() {
  //   return (!mLeftPivotPiston.get() && !mRightPivotPiston.get());
  // }

  // private boolean extendedPosition() {
  //   return (mLeftPivotPiston.get() && mRightPivotPiston.get());
  // }

  public void togglePivot(){
    System.out.println("toggle pivot");
    mLeftPivotPiston.toggle();
    mRightPivotPiston.toggle();
  }

   public double getArmAngle(){
     return currentAngle;
   }
  @Override
  public void periodic() {
    // System.out.println(getEncoderDistance());
    currentAngle = ElementUnits.ticksToRotations(left.getSelectedSensorPosition(), Constants.TalonFXEncoderPPR);
    currentAngle *= Constants.ArmConstants.kGearRatio; // gets rotation of the arm from rotations of the motors
    currentAngle *= 360; // gets the current angle of the arm in degrees
     SmartDashboard.putNumber("Arm Angle", getArmAngle());
  }
}
