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
                // specify the position and branch then set pose and state, L1
                if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_ab)) {
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_RightBranch)) {
                        coralScoreGoalPose = Settings.CoralRightAB;
                        updateRobotState()
                    }
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_LeftBranch)) {
                        coralScoreGoalPose = Settings.CoralLeftAB;
                        updateRobotState()
                    }                
                }

                if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_cd)) {
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_RightBranch)) {
                        coralScoreGoalPose = Settings.CoralRightCD;
                        updateRobotState()
                    }
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_LeftBranch)) {
                        coralScoreGoalPose = Settings.CoralLeftCD;
                        updateRobotState()
                    }                
                }

                if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_ef)) {
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_RightBranch)) {
                        coralScoreGoalPose = Settings.CoralRightEF;
                        updateRobotState()
                    }
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_LeftBranch)) {
                        coralScoreGoalPose = Settings.CoralLeftEF;
                        updateRobotState()
                    }                
                }

                if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_gh)) {
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_RightBranch)) {
                        coralScoreGoalPose = Settings.CoralRightGH;
                        updateRobotState()
                    }
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_LeftBranch)) {
                        coralScoreGoalPose = Settings.CoralLeftGH;
                        updateRobotState()
                    }                
                }

                if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_ij)) {
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_RightBranch)) {
                        coralScoreGoalPose = Settings.CoralRightIJ;
                        updateRobotState()
                    }
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_LeftBranch)) {
                        coralScoreGoalPose = Settings.CoralLeftIJ;
                        updateRobotState()
                    }                
                }

                if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_kl)) {
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_RightBranch)) {
                        coralScoreGoalPose = Settings.CoralRightKL;
                        updateRobotState()
                    }
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_LeftBranch)) {
                        coralScoreGoalPose = Settings.CoralLeftKL;
                        updateRobotState()
                    }                
                }

                break;
            default:
                break;
        }

    public void updateRobotState() {
            if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral1)) {
                robotState = RobotStates.coralScore1;
            } else if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral2)) {
                robotState = RobotStates.coralScore2;
            } else if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral3)) {
                robotState = RobotStates.coralScore3;
            } else if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral4)) {
                robotState = RobotStates.coralScore4;
            }
    }
    }
}
