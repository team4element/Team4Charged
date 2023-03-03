package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Arm;
import frc.robot.controllers.OperatorController;
import frc.robot.ElementMath;

public class ArmControl extends CommandBase {
  private final Arm m_arm;
  OperatorController mOperatorController;
  
  public ArmControl(Arm arm, OperatorController controller) {
    this.m_arm = arm;
    this.mOperatorController = controller;

    addRequirements(this.m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = ElementMath.cubeInput(this.mOperatorController.getThrottle());
    m_arm.setArmPower(power);
    // m_arm.getYAxis();
    // m_arm.getMotorOutput();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.resetSensors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
