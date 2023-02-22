package frc.robot.controllers;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.XboxController;

public class OperatorController {
    // private double triggerTolerance = .25;
    private final XboxController mController;

    public OperatorController(){
        mController = new XboxController(Constants.ControllerConstants.kOperatorControllerPort);
    }

    public boolean getCompressorToggle(){
        return mController.getBButton();
    }
}
