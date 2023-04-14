package frc.robot.controllers;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.XboxController;

public class DriverController {
    private double triggerTolerance = .25;
    private final XboxController mController;

    public DriverController(){
        mController = new XboxController(Constants.ControllerConstants.kDriverControllerPort);
    }

    public double getThrottle(){
        return -mController.getLeftY();
    }

    public double getTurn(){
        return -mController.getRightX();
    }

    public boolean getSlowTurnLeft(){
        return mController.getLeftTriggerAxis() > triggerTolerance;
    }

    public boolean getSlowTurnRight(){
        return mController.getRightTriggerAxis() > triggerTolerance;
    }
    
    public boolean getRotate(){
        return mController.getYButton();
    }

    public boolean getBalance(){
        return mController.getXButton();
    }

    public boolean getSlowDrive(){
        return mController.getBButton();
    }
}
