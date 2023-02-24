// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Intake extends SubsystemBase {
  // Declaring Motors
  private WPI_VictorSPX left;
  private WPI_VictorSPX right;

  // Declaring Solenoids
  public static Solenoid mLeftIntakePiston;
  public static Solenoid mRightIntakePiston;

  public Intake() {
    // Instantiating Motors
    left = new WPI_VictorSPX(Constants.IntakeConstants.kLeftMotor);
    right = new WPI_VictorSPX(Constants.IntakeConstants.kRightMotor);

    // Instantiating Solenoids
    mLeftIntakePiston = new Solenoid(3, PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.kLeftIntakeSolenoid);
    mRightIntakePiston = new Solenoid(4, PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.kRightIntakeSolenoid);
  }

  public void setIntakeForwardPower(double forwardPower){
    left.set(VictorSPXControlMode.PercentOutput, forwardPower);
    right.set(VictorSPXControlMode.PercentOutput, forwardPower);
  }

  public void setIntakeReversePower(double reversePower){
    left.set(VictorSPXControlMode.PercentOutput, reversePower);
    right.set(VictorSPXControlMode.PercentOutput, reversePower);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
