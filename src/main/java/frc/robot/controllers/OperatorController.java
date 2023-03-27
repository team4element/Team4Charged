package frc.robot.controllers;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.XboxController;

public class OperatorController {
    private double triggerTolerance = .25;
    private final XboxController mController;

    public OperatorController(){
        mController = new XboxController(Constants.ControllerConstants.kOperatorControllerPort);
    }

    public boolean getCompressorToggle(){
        return mController.getRightStickButton();
    }

    public boolean getIntakeForward(){
        return mController.getAButton();
    }

    public boolean getIntakeReverse(){
        return mController.getBButton();
    }

    public boolean getDeployClaw(){
        return mController.getXButton();
    }

    public boolean getPivot(){
        return mController.getYButton();
    }

    public double getThrottle(){
        // Up is negative on the controller by default
        return -mController.getLeftY();
    }

    public boolean highConePosition(){
        return mController.getLeftBumper();
    }

    public boolean highCubePosition(){
        return mController.getLeftTriggerAxis() > triggerTolerance;
    }

    public boolean midCubePosition(){
        return mController.getRightTriggerAxis() > triggerTolerance;
    }

    public boolean shelfPosition(){
        return mController.getRightBumper();
    }
}