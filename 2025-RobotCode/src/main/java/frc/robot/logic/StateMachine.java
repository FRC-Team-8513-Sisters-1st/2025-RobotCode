package frc.robot.logic;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.logic.Enums.RobotStates;

public class StateMachine {

    Robot thisRobot;

    RobotStates robotState = RobotStates.driving;
    Pose2d coralScoreGoalPose;

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
                if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_ab) || (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_cd)) || (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_ef)) || (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_gh)) || (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_ij)) || (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_kl))) {
                    storeCoralScore = true;
                    if (thisRobot.teleopController.driverXboxController
                            .getRawButtonPressed(Settings.buttonId_RightBranch)) {
                        robotState = RobotStates.coralScore1;
                    }
                    if (thisRobot.teleopController.driverXboxController
                            .getRawButtonPressed(Settings.buttonId_LeftBranch)) {
                        robotState = RobotStates.coralScore1;
                    }               
                }

                if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_ab) || (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_cd)) || (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_ef)) || (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_gh)) || (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_ij)) || (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_kl))) {
                    storeCoralScore = true;
                    if (thisRobot.teleopController.driverXboxController
                            .getRawButtonPressed(Settings.buttonId_RightBranch)) {
                        robotState = RobotStates.coralScore2;
                    }
                    if (thisRobot.teleopController.driverXboxController
                            .getRawButtonPressed(Settings.buttonId_LeftBranch)) {
                        robotState = RobotStates.coralScore2;
                    }               
                }

                if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_ab) || (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_cd)) || (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_ef)) || (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_gh)) || (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_ij)) || (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_kl))) {
                    storeCoralScore = true;
                    if (thisRobot.teleopController.driverXboxController
                            .getRawButtonPressed(Settings.buttonId_RightBranch)) {
                        robotState = RobotStates.coralScore3;
                        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Algae2)) {
                            robotState = RobotStates.algaeIntakeL2;
                        }
                    }
                    if (thisRobot.teleopController.driverXboxController
                            .getRawButtonPressed(Settings.buttonId_LeftBranch)) {
                        robotState = RobotStates.coralScore3;
                    }               
                }

                if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_ab) || (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_cd)) || (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_ef)) || (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_gh)) || (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_ij)) || (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_kl))) {
                    storeCoralScore = true;
                    if (thisRobot.teleopController.driverXboxController
                            .getRawButtonPressed(Settings.buttonId_RightBranch)) {
                        robotState = RobotStates.coralScore4;
                        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Algae3)) {
                            robotState = RobotStates.algaeIntakeL3;
                        }
                    }
                    if (thisRobot.teleopController.driverXboxController
                            .getRawButtonPressed(Settings.buttonId_LeftBranch)) {
                        robotState = RobotStates.coralScore4;
                    }               
                }

                break;
            default:
                break;
        }
    }
}
