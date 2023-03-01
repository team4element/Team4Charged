package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.VisionTracker;

public class RotateToTarget extends CommandBase {

    PIDController anglePID;

    // this -> current instance of the class

    private final DriveTrain m_drive;
    private final VisionTracker m_vision;

    private static final double tolerance = 0.2;

    public RotateToTarget(DriveTrain drive, VisionTracker vision) {
        anglePID = new PIDController(Constants.DriveConstants.kAngleP, Constants.DriveConstants.kAngleI, Constants.DriveConstants.kAngleD);
        this.m_drive = drive;
        m_vision = vision;

        anglePID.setTolerance(tolerance);

        addRequirements(this.m_drive, this.m_vision);
    }

    @Override
    public void initialize() {
        anglePID.setSetpoint(0.0);
        this.m_drive.setPower(0.0, 0.0);
    }

    @Override
    public void execute() {
        double power = anglePID.calculate(m_vision.getAngle());
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
