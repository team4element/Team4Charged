package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class RotateToAngle extends CommandBase {

    PIDController anglePID;

    // this -> current instance of the class

    private final double m_angle;
    private final DriveTrain m_drive;

    private static final double tolerance = 2.0;

    public RotateToAngle(DriveTrain drive, double angle) {
        anglePID = new PIDController(Constants.DriveConstants.kAngleP, Constants.DriveConstants.kAngleI, Constants.DriveConstants.kAngleD);
        this.m_angle = angle;
        this.m_drive = drive;

        anglePID.setTolerance(tolerance);

        addRequirements(this.m_drive);
    }

    @Override
    public void initialize() {
        anglePID.setSetpoint(this.m_angle);
        this.m_drive.setPower(0.0, 0.0);
    }

    @Override
    public void execute() {
        double power = anglePID.calculate(this.m_drive.getGyro());
        this.m_drive.setPower(-power, power);
    }

    @Override
    public void end(boolean interrupted) {
        this.m_drive.setPower(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return anglePID.atSetpoint();
    }
}
