// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;
import frc.robot.controllers.DriverController;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  //Declare Motor Objects
  private WPI_TalonFX leftFront;
  private WPI_TalonFX leftBack;

  private WPI_TalonFX rightFront;
  private WPI_TalonFX rightBack;

  private AHRS navX;

  public static DriverController mDriverController = new DriverController();

  public DriveTrain() {
    //Define Motor Objects
    leftFront = new WPI_TalonFX(Constants.DriveConstants.kLeftFrontMotor);
    leftBack = new WPI_TalonFX(Constants.DriveConstants.kLeftBackMotor);
    
    rightFront = new WPI_TalonFX(Constants.DriveConstants.kRightFrontMotor);
    rightBack = new WPI_TalonFX(Constants.DriveConstants.kRightBackMotor);

    navX = new AHRS(SPI.Port.kMXP);

    //Make motors Follow the Leader 
    leftFront.follow(leftBack);

    rightFront.follow(rightBack);

    leftBack.setInverted(true);
    leftFront.setInverted(true);

  }

  public void setPower(double leftPower, double rightPower){
    leftBack.set(TalonFXControlMode.PercentOutput, leftPower);
    rightBack.set(TalonFXControlMode.PercentOutput, rightPower);
  }

  public void resetSensors(){
    navX.reset();
  }

  public double getGyro(){
    return navX.getYaw();
  }
  
  public boolean rotate(){
     return mDriverController.getRotate();
  }
  @Override
  public void periodic() {
    System.out.println(getGyro());
  }
}
