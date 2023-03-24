// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class HoldArmPosition extends CommandBase {

  private PIDController armPID;

  private final double m_angle;
  private final Arm m_arm;

  private static final double tolerance = 2;

  ArmFeedforward feedForward = new ArmFeedforward(Constants.ArmConstants.kS, Constants.ArmConstants.kG,
      Constants.ArmConstants.kV, Constants.ArmConstants.kA);
  
  public HoldArmPosition(Arm arm, double angle) {

    this.m_angle = angle;
    this.m_arm = arm;

    armPID = new PIDController(Constants.ArmConstants.kArmP, Constants.ArmConstants.kArmI, Constants.ArmConstants.kArmD, feedForward.calculate(Units.degreesToRadians(m_angle), 2, 2));

    armPID.setTolerance(this.m_arm.armAngleToTicks(tolerance));
      
    addRequirements(this.m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.m_arm.setArmPower(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = armPID.calculate(this.m_arm.armAngleToTicks(m_angle));
    this.m_arm.setArmPower(power);
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
