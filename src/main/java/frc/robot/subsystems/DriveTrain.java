// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  //Declare Motor Objects
  private WPI_VictorSPX leftFront;
  private WPI_VictorSPX leftMiddle;
  private WPI_TalonSRX leftBack;

  private WPI_VictorSPX rightFront;
  private WPI_TalonSRX rightMiddle;
  private WPI_VictorSPX rightBack;

  private AHRS navX;

  public DriveTrain() {
    //Define Motor Objects
    leftFront = new WPI_VictorSPX(Constants.DriveConstants.kLeftFrontMotor);
    leftMiddle = new WPI_VictorSPX(Constants.DriveConstants.kLeftMiddleMotor);
    leftBack = new WPI_TalonSRX(Constants.DriveConstants.kLeftBackMotor);
    
    rightFront = new WPI_VictorSPX(Constants.DriveConstants.kRightFrontMotor);
    rightMiddle = new WPI_TalonSRX(Constants.DriveConstants.kRightMiddleMotor);
    rightBack = new WPI_VictorSPX(Constants.DriveConstants.kRightBackMotor);

    navX = new AHRS(SPI.Port.kMXP);

    //Make motors Follow the Leader 
    leftMiddle.follow(leftBack);
    leftFront.follow(leftBack);

    rightBack.follow(rightMiddle);
    rightFront.follow(rightMiddle);

    leftBack.setInverted(true);
    leftMiddle.setInverted(true);
    leftFront.setInverted(true);

  }

  public void setPower(double leftPower, double rightPower){
    leftBack.set(TalonSRXControlMode.PercentOutput, leftPower);
    rightMiddle.set(TalonSRXControlMode.PercentOutput, rightPower);
  }
  
  public void resetSensors(){
    navX.reset();
  }

  public double getGyro(){
    return navX.getYaw();
  }
  
  @Override
  public void periodic() {
    System.out.println(getGyro());
  }

}


