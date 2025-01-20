package frc.robot.logic;

import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.logic.Enums.ElevatorStates;
import frc.robot.logic.Enums.RobotStates;

public class StateMachine {

    Robot thisRobot;

    RobotStates robotState = RobotStates.driving;
    boolean storeCoralScore;

    public StateMachine(Robot thisRobotIn) {

        thisRobot = thisRobotIn;
    }

    public void enumRobotState() {

        switch (robotState) {
            case driving:
                if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_CoralIntake)) {
                    robotState = RobotStates.coralIntakeFeederSt;
                }
                if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Processor)) {
                    robotState = RobotStates.algaeScoreProcessor;
                }
                if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_Climb)) {
                    robotState = RobotStates.climbDeep;
                }
                if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_ab)) {
                    storeCoralScore = true;
                    if (thisRobot.teleopController.driverXboxController
                            .getRawButtonPressed(Settings.buttonId_RightBranch)) {
                        robotState = RobotStates.coral1;
                    }
                    if (thisRobot.teleopController.driverXboxController
                            .getRawButtonPressed(Settings.buttonId_LeftBranch)) {
                        robotState = RobotStates.coral1;
                    }

                }
                break;
            default:
                break;
        }
    }
}
