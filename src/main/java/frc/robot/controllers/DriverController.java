package frc.robot.controllers;

import frc.robot.Constants;
import frc.robot.ElementMath;

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
        return ElementMath.squareInput(mController.getRightX()/-1.5);
    }

    public boolean getRotate(){
        return mController.getYButton();
    }
}
