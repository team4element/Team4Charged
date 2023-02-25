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
        return mController.getBButton();
    }

    public boolean getIntakeForward(){
        return mController.getLeftBumper();
    }

    public boolean getIntakeReverse(){
        return mController.getLeftTriggerAxis() > triggerTolerance;
    }

    public boolean getDeployClaw(){
        return mController.getXButtonPressed();
    }
}