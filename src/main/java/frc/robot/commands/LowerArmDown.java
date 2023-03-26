// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class LowerArmDown extends CommandBase {

  private PIDController armPID;

  private final Arm m_arm;
  
  private static final double tolerance = .25;

  ArmFeedforward feedForward = new ArmFeedforward(Constants.ArmConstants.kS, Constants.ArmConstants.kG,
      Constants.ArmConstants.kV, Constants.ArmConstants.kA);
  
  public LowerArmDown(Arm arm) {

    this.m_arm = arm;

    addRequirements(this.m_arm);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (this.m_arm.getPivotState()){
      armPID = new PIDController(Constants.ArmConstants.kArmP, Constants.ArmConstants.kArmI, Constants.ArmConstants.kArmD, feedForward.calculate(Math.cos(Units.degreesToRadians(Constants.ArmConstants.kPivotedAngle)), 2, 2));
    } else {
      armPID = new PIDController(Constants.ArmConstants.kArmP, Constants.ArmConstants.kArmI, Constants.ArmConstants.kArmD, feedForward.calculate(Math.cos(Units.degreesToRadians(Constants.ArmConstants.kBaseAngle)), 2, 2));
    }
  
    // armPID = new PIDController(Constants.ArmConstants.kArmP, Constants.ArmConstants.kArmI, Constants.ArmConstants.kArmD);
  
    armPID.setTolerance(this.m_arm.armAngleToTicks(tolerance));
      
    this.m_arm.setArmPower(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (this.m_arm.getPivotState()){
      double power = MathUtil.clamp(armPID.calculate(this.m_arm.getEncoderDistance(), this.m_arm.armAngleToTicks(Constants.ArmConstants.kPivotedAngle)), -0.5, 0.9);
      this.m_arm.setArmPower(power);
    } else {
      double power = MathUtil.clamp(armPID.calculate(this.m_arm.getEncoderDistance(), this.m_arm.armAngleToTicks(Constants.ArmConstants.kBaseAngle)), -0.5, 0.9);
      this.m_arm.setArmPower(power);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}