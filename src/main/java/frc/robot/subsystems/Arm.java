// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.controllers.OperatorController;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  // Declaring Motors
  private WPI_TalonFX leftFront;
  private WPI_TalonFX leftBack;

  private WPI_TalonFX rightFront;
  private WPI_TalonFX rightBack;

  // Declaring Solenoids
  private static Solenoid mLeftArmPiston;
  private static Solenoid mRightArmPiston;

  // Declaring Controller
  public static OperatorController mOperatorController = new OperatorController();

  // Declaring Compressor
  public static Compressor mCompressor;

  // Declaring Encoder

  private static Encoder mEncoder;

  public Arm() {
    // Instantiating Motors
    leftFront = new WPI_TalonFX(Constants.ArmConstants.kLeftFrontMotor);
    leftBack = new WPI_TalonFX(Constants.ArmConstants.kLeftBackMotor);

    rightFront = new WPI_TalonFX(Constants.ArmConstants.kRightFrontMotor);
    rightBack = new WPI_TalonFX(Constants.ArmConstants.kRightBackMotor);

    // Make motors follow the leader
    leftBack.follow(leftFront);
    rightBack.follow(rightFront);

    //Instantiating Solenoids
    mLeftArmPiston = new Solenoid(0, PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.kLeftArmSolenoid);
    mRightArmPiston = new Solenoid(0, PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.kRightArmSolenoid);

    //Instantiating Compressor
    mCompressor = new Compressor(Constants.PneumaticsConstants.kCompressorID, PneumaticsModuleType.CTREPCM);

    mEncoder = new Encoder(0, 1);
  }

  public boolean toggleCompressor(){
    return mOperatorController.getCompressorToggle();
  }

  public double getEncoderDistance(){
    return mEncoder.getDistance();
  }
  
  public void setArmPower(double power){
    leftFront.set(TalonFXControlMode.PercentOutput, power);
    rightFront.set(TalonFXControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
