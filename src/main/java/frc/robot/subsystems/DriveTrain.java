// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  //Declare Motor Objects
  private WPI_VictorSPX leftFront;
  private WPI_VictorSPX leftMiddle;
  private WPI_TalonSRX leftBack;

  private WPI_VictorSPX rightFront;
  private WPI_VictorSPX rightMiddle;
  private WPI_TalonSRX rightBack;

  public DriveTrain() {
    //Define Motor Objects
    leftFront = new WPI_VictorSPX(1);
    leftMiddle = new WPI_VictorSPX(2);
    leftBack = new WPI_TalonSRX(3);
    
    rightFront = new WPI_VictorSPX(4);
    rightMiddle = new WPI_VictorSPX(5);
    rightBack = new WPI_TalonSRX(6);

    //Make motors Follow the Leader 
    leftMiddle.follow(leftBack);
    leftFront.follow(leftBack);

    rightMiddle.follow(rightBack);
    rightFront.follow(leftBack);

    leftBack.setInverted(true);
    leftMiddle.setInverted(true);
    leftFront.setInverted(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double leftPower, double rightPower){
    leftBack.set(TalonSRXControlMode.PercentOutput, leftPower);
    rightBack.set(TalonSRXControlMode.PercentOutput, rightPower);
  }
}
