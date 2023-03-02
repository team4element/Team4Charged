package frc.robot.controllers;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.XboxController;

public class DriverController {
    private final XboxController mController;

    public DriverController(){
        mController = new XboxController(Constants.ControllerConstants.kDriverControllerPort);
    }

    public double getThrottle(){
        return mController.getLeftY();
    }

    public double getTurn(){
        return mController.getRightX();
    }

    public boolean getRotate(){
        return mController.getYButton();
    }
}
