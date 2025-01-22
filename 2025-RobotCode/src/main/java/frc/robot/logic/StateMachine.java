package frc.robot.logic;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.logic.Enums.RobotStates;

public class StateMachine {

    Robot thisRobot;

    RobotStates robotState = RobotStates.driving;
    Pose2d coralScoreGoalPose;
    Pose2d goalOperatorReefPose;

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

                // store if operator hits reef button
                if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_ab)) {
                    goalOperatorReefPose = Settings.AB;
                }
                if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_cd)) {
                    goalOperatorReefPose = Settings.CD;
                }
                if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_ef)) {
                    goalOperatorReefPose = Settings.EF;
                }
                if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_gh)) {
                    goalOperatorReefPose = Settings.GH;
                }
                if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_ij)) {
                    goalOperatorReefPose = Settings.IJ;
                }
                if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_kl)) {
                    goalOperatorReefPose = Settings.KL;
                }

                // driver selects branch and scores
                if (goalOperatorReefPose == Settings.AB) {
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_RightBranch)) {
                        coralScoreGoalPose = Settings.CoralRightAB;
                        updateRobotScoreState();
                    }
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_LeftBranch)) {
                        coralScoreGoalPose = Settings.CoralLeftAB;
                        updateRobotScoreState();
                    }                
                }

                if (goalOperatorReefPose == Settings.CD) {
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_RightBranch)) {
                        coralScoreGoalPose = Settings.CoralRightCD;
                        updateRobotScoreState();
                    }
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_LeftBranch)) {
                        coralScoreGoalPose = Settings.CoralLeftCD;
                        updateRobotScoreState();
                    }                
                }

                if (goalOperatorReefPose == Settings.EF) {
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_RightBranch)) {
                        coralScoreGoalPose = Settings.CoralRightEF;
                        updateRobotScoreState();
                    }
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_LeftBranch)) {
                        coralScoreGoalPose = Settings.CoralLeftEF;
                        updateRobotScoreState();
                    }                
                }

                if (goalOperatorReefPose == Settings.GH) {
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_RightBranch)) {
                        coralScoreGoalPose = Settings.CoralRightGH;
                        updateRobotScoreState();
                    }
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_LeftBranch)) {
                        coralScoreGoalPose = Settings.CoralLeftGH;
                        updateRobotScoreState();
                    }                
                }

                if (goalOperatorReefPose == Settings.IJ) {
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_RightBranch)) {
                        coralScoreGoalPose = Settings.CoralRightIJ;
                        updateRobotScoreState();
                    }
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_LeftBranch)) {
                        coralScoreGoalPose = Settings.CoralLeftIJ;
                        updateRobotScoreState();
                    }                
                }

                if (goalOperatorReefPose == Settings.KL) {
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_RightBranch)) {
                        coralScoreGoalPose = Settings.CoralRightKL;
                        updateRobotScoreState();
                    }
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_LeftBranch)) {
                        coralScoreGoalPose = Settings.CoralLeftKL;
                        updateRobotScoreState();
                    }                
                }

                break;
            default:
                break;
        }
    }

    public void updateRobotScoreState() {
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
