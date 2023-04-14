// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import frc.robot.Constants;
import frc.robot.commands.*;

import frc.robot.controllers.DriverController;
// import frc.robot.controllers.OperatorController;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final static DriveTrain m_driveTrain = new DriveTrain();
  public final static Arm m_arm = new Arm();
  public final static Intake m_intake = new Intake();

  private final DriverController m_driverController = new DriverController();
//   private final OperatorController m_operatorController = new OperatorController();

  private final Command TaxiMode = new TaxiAuto(m_driveTrain);
  private final Command ScoreLowMode = new Score(m_intake).withTimeout(1);
  private final Command ScoreMidMode = Commands.sequence(
    new HoldArmPosition(m_arm, 74.5).withTimeout(2),
    new ProfiledDriveToPosition(m_driveTrain, 11.5, .5).withTimeout(2),
    new ToggleClaw(m_intake),
    new ProfiledDriveToPosition(m_driveTrain, -10, .5).withTimeout(2),
    new LowerArmDown(m_arm).withTimeout(1.5));
  private final Command ScoreHighMode = Commands.sequence(
    new HighConePosition(m_arm).withTimeout(2),
    new ProfiledDriveToPosition(m_driveTrain, 23.5, 1).withTimeout(2),
    new ToggleClaw(m_intake),
    new ProfiledDriveToPosition(m_driveTrain, -24, 1).withTimeout(2),
    new TogglePivot(m_arm),
    new LowerArmDown(m_arm).withTimeout(1.5));
  private final Command DoNothingMode = new DoNothingMode();
  private final Command ScoreLowAndTaxiMode = Commands.sequence(
      new Score(m_intake),
      new TaxiAuto(m_driveTrain));
  private final Command ScoreMidAndTaxiMode = new ScoreMidAndTaxiAuto(m_driveTrain, m_arm, m_intake);
  private final Command ScoreHighAndTaxiMode = new ScoreHighAndTaxiAuto(m_driveTrain, m_arm, m_intake);
  private final Command ScoreLowAndTaxiAndBalanceMode = new ScoreLowAndBalance(m_driveTrain, m_intake);
  private final Command BalanceMode = Commands.sequence(
    new ProfiledDriveToPosition(m_driveTrain, -82, 1).withTimeout(3),
    new Balance(m_driveTrain));
  private final Command ScoreMidAndTaxiAndBalanceMode = new ScoreMidAndBalance(m_driveTrain, m_arm, m_intake);
  private final Command ScoreHighAndTaxiAndBalanceMode = new ScoreHighAndBalance(m_driveTrain, m_arm, m_intake);
  private final Command TaxiAndBalanceMode = Commands.sequence(
      new ProfiledDriveToPosition(m_driveTrain, -148, 1).withTimeout(5),
      new ProfiledDriveToPosition(m_driveTrain, 68, 1).withTimeout(3),
      new Balance(m_driveTrain));
  private final Command NewBalanceMode = new Balance(m_driveTrain);
  private final Command ScoreLowAndIntakeMode = new ScoreLowAndIntake(m_driveTrain, m_intake);
  private final Command ScoreMidAndIntakeMode = new ScoreMidAndIntake(m_driveTrain, m_arm, m_intake);
  private final Command ScoreHighAndIntakeMode = new ScoreHighAndIntake(m_driveTrain, m_arm, m_intake);
  private final Command ProfiledPID = new ProfiledDriveToPosition(m_driveTrain, -160);
  // private final Command Straight = new FollowTrajectoryPP(m_driveTrain);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_driveTrain.setDefaultCommand(new Drive(m_driveTrain, m_driverController));
    m_arm.setDefaultCommand(new LowerArmDown(m_arm));

    // Auto Modes
    m_chooser.setDefaultOption("Score Mid and Taxi Auto", ScoreMidAndTaxiMode);
    m_chooser.addOption("Do Nothing Auto", DoNothingMode);
    m_chooser.addOption("Score Low Auto", ScoreLowMode);
    m_chooser.addOption("Score Mid Auto", ScoreMidMode);
    m_chooser.addOption("Score High Auto", ScoreHighMode);
    m_chooser.addOption("Taxi Auto", TaxiMode);
    m_chooser.addOption("Balance Auto", BalanceMode);
    m_chooser.addOption("Score Low And Taxi Auto", ScoreLowAndTaxiMode);
    m_chooser.addOption("Score High And Taxi Auto", ScoreHighAndTaxiMode);
    m_chooser.addOption("Score Low, Taxi, and Balance Auto", ScoreLowAndTaxiAndBalanceMode);
    m_chooser.addOption("Score Mid, Taxi, and Balance Auto", ScoreMidAndTaxiAndBalanceMode);
    m_chooser.addOption("Score High, Taxi, and Balance Auto", ScoreHighAndTaxiAndBalanceMode);
    m_chooser.addOption("Taxi and Balance Auto", TaxiAndBalanceMode);
    m_chooser.addOption("New Balance Auto", NewBalanceMode);
    m_chooser.addOption("Score Low and Intake Auto", ScoreLowAndIntakeMode);
    m_chooser.addOption("Score Mid and Intake Auto", ScoreMidAndIntakeMode);
    m_chooser.addOption("Score High and Intake Auto", ScoreHighAndIntakeMode);
    m_chooser.addOption("Profiled PID", ProfiledPID);
    SmartDashboard.putData(m_chooser);
  }

  public void enableBrakeMode() {
    m_driveTrain.setBrakeMode();
  }

  public void disableBrakeMode() {
    m_driveTrain.setCoastMode();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
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

    // Run HoldArmPosition Command for High Cone Position When Operator Left Bumper
    // is Pressed
    new Trigger(m_arm::getHighConePosition)
        .onTrue(new HighConePosition(m_arm))
        .onFalse(new TogglePivot(m_arm));

    // Run HoldArmPosition Command for High Cube Position when Operator Left Trigger
    // is Pressed
    new Trigger(m_arm::getHighCubePosition)
        .onTrue(new HoldArmPosition(m_arm, 73.5))
        .onFalse(new LowerArmDown(m_arm));

    // Run HoldArmPosition Command for Mid Cube Position when Operator Right Trigger
    // is Pressed
    new Trigger(m_arm::getMidCubePosition)
        .onTrue(new HoldArmPosition(m_arm, 55))
        .onFalse(new LowerArmDown(m_arm));

    // Run HoldArmPosition Command for Shelf Position when Operator Right Bumper is
    // Pressed
    new Trigger(m_arm::getShelfPosition)
        .onTrue(new HoldArmPosition(m_arm, 70))
        .onFalse(new LowerArmDown(m_arm));

    // Run TogglePivot Command when Operator A Button is Pressed
    new Trigger(m_arm::getTogglePivot)
        .onTrue(new TogglePivot(m_arm));
    // .onTrue(new RaiseArmAndPivot(m_arm, 10));

    // Run ToggleCompressor Command when Operator B Button is Pressed
    new Trigger(m_arm::toggleCompressor)
        .onTrue(new ToggleCompressor(m_arm));

    // Run IntakeForward Command when Operator Left Bumper is Held
    new Trigger(m_intake::intakeForward)
        .whileTrue(new IntakeForward(m_intake));

    // Run IntakeReverse Command when Operator Left Trigger is Held
    new Trigger(m_intake::intakeReverse)
        .whileTrue(new IntakeReverse(m_intake));

    // Run ToggleClaw Command when Operator X Button is Pressed
    new Trigger(m_intake::toggleClaw)
        .onTrue(new ToggleClaw(m_intake));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  public static DriveTrain getDriveTrainSubsystem() {
    return m_driveTrain;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
