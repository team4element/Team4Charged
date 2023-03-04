// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import frc.robot.Constants;
import frc.robot.commands.Autos;
import frc.robot.commands.RotateToAngle;
import frc.robot.commands.Drive;
import frc.robot.commands.ToggleCompressor;
import frc.robot.commands.IntakeForward;
import frc.robot.commands.IntakeReverse;
import frc.robot.commands.ToggleClaw;
import frc.robot.commands.ArmControl;
import frc.robot.commands.TogglePivot;
import frc.robot.commands.ArmToHigh;
import frc.robot.commands.ArmToMid;
import frc.robot.commands.SlowTurnLeft;
import frc.robot.commands.SlowTurnRight;

import frc.robot.controllers.DriverController;
import frc.robot.controllers.OperatorController;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public static DriveTrain m_driveTrain = new DriveTrain();
  public static Arm m_arm = new Arm();
  public static Intake m_intake = new Intake();

  private final DriverController m_driverController = new DriverController();
  private final OperatorController m_operatorController = new OperatorController();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_driveTrain.setDefaultCommand(new Drive(m_driveTrain, m_driverController));
    m_arm.setDefaultCommand(new ArmControl(m_arm, m_operatorController));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Run RotateToAngle Command when Driver Y Button is Pressed
    new Trigger(m_driveTrain::rotate)
      .onTrue(new RotateToAngle(m_driveTrain, 90));

    // Run SlowTurnLeft Command when Driver Left Trigger is Pressed
    new Trigger(m_driveTrain::slowTurnLeft)
      .whileTrue(new SlowTurnLeft(m_driveTrain, m_driverController));

    // Run SlowTurnRight Command when Driver Right Trigger is Pressed
    new Trigger(m_driveTrain::slowTurnRight)
      .whileTrue(new SlowTurnRight(m_driveTrain, m_driverController));

    // Run ArmToMid Command when Operator Right Bumper is Pressed
    new Trigger(m_arm::getMidPosition)
      .onTrue(new ArmToMid(m_arm, 0));
    // TODO: Set distance for ArmToMid

    // Run ArmToHigh Command when Operator Right Trigger is Pressed
    new Trigger(m_arm::getHighPosition)
      .onTrue(new ArmToHigh(m_arm, 72450));
    
    // Run TogglePivot Command when Operator A Button is Pressed
    new Trigger(m_arm::getTogglePivot)
    .onTrue(new TogglePivot(m_arm));

    // Run ToggleCompressor Command when Operator B Button is Pressed
    new Trigger(m_arm::toggleCompressor)
      .whileTrue(new ToggleCompressor(m_arm));

    // Run IntakeForward Command when Operator Left Bumper is Held
    new Trigger(m_intake::intakeForward)
      .whileTrue(new IntakeForward(m_intake));

    // Run IntakeReverse Command when Operator Left Trigger is Held
    new Trigger(m_intake::intakeReverse)
      .whileTrue(new IntakeReverse(m_intake));
    
    // Run ToggleClaw Command when Operator X Button is Pressed
    new Trigger(m_intake::toggleClaw)
      .onTrue(new ToggleClaw(m_intake));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.taxiAuto(m_driveTrain);
  }
}
