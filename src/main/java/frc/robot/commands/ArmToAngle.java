 package frc.robot.commands;

 import edu.wpi.first.wpilibj2.command.CommandBase;

 import frc.robot.ElementUnits;
 import frc.robot.subsystems.Arm;
 import frc.robot.Constants;

 public class ArmToAngle extends CommandBase {
   private final Arm m_arm;

   private static double m_angle;
   private double desiredTicks;
     private static final double tolerance = 10; // TODO set tolerance

   public ArmToAngle(Arm arm, double angle) {
     this.m_angle = angle;
     this.m_arm = arm;

     addRequirements(this.m_arm);
   }

   // Called when the command is initially scheduled.
   @Override
   public void initialize() {
       this.m_arm.setArmPower(0);
       desiredTicks = m_angle / 360; // converts from degrees to arm rotation
       desiredTicks = m_angle / Constants.ArmConstants.kGearRatio; // converts from arm rotation to motor rotation
       desiredTicks = ElementUnits.rotationsToTicks(desiredTicks, Constants.TalonFXEncoderPPR); // converts from motor rotation to encoder ticks

       // sets the TalonFX position PID to reach this setpoint (should only need one call)
       m_arm.setArmPosition(desiredTicks);
   }

   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute() {
       // TODO if calling once doesnt work uncomment line and use this
        // m_arm.setArmPosition(desiredTicks);
   }

   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
     this.m_arm.setArmPower(0);
   }

   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
     return Math.abs(m_arm.getArmAngle() - m_angle) < tolerance;
   }
 }
