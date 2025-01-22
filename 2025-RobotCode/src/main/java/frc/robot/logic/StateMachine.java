package frc.robot.logic;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.logic.Enums.ElevatorStates;
import frc.robot.logic.Enums.RobotStates;
import frc.robot.logic.Enums.SideOfReef;

public class StateMachine {

    Robot thisRobot;

    RobotStates robotState = RobotStates.driving;
    Pose2d coralScoreGoalPose;
    SideOfReef operatorChosenSideOfReef;
    ElevatorStates scoreCoralGoalLevel;
    boolean isInReefZone = false;

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
                    operatorChosenSideOfReef = SideOfReef.AB;
                }
                if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_cd)) {
                    operatorChosenSideOfReef = SideOfReef.CD;
                }
                if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_ef)) {
                    operatorChosenSideOfReef = SideOfReef.EF;
                }
                if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_gh)) {
                    operatorChosenSideOfReef = SideOfReef.GH;
                }
                if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_ij)) {
                    operatorChosenSideOfReef = SideOfReef.IJ;
                }
                if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_kl)) {
                    operatorChosenSideOfReef = SideOfReef.KL;
                }

                // driver selects l or r branch and co-pilot stored info is run
                if (operatorChosenSideOfReef == SideOfReef.AB) {
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_RightBranch)) {
                        coralScoreGoalPose = Settings.coralRightAB;
                        scoreCoralOnLevel();
                    } else if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_LeftBranch)) {
                        coralScoreGoalPose = Settings.coralLeftAB;
                        scoreCoralOnLevel();
                    }                
                }

                if (operatorChosenSideOfReef == SideOfReef.CD) {
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_RightBranch)) {
                        coralScoreGoalPose = Settings.coralRightCD;
                        scoreCoralOnLevel();
                    } else if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_LeftBranch)) {
                        coralScoreGoalPose = Settings.coralLeftCD;
                        scoreCoralOnLevel();
                    }                
                }

                if (operatorChosenSideOfReef == SideOfReef.EF) {
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_RightBranch)) {
                        coralScoreGoalPose = Settings.coralRightEF;
                        scoreCoralOnLevel();
                    } else if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_LeftBranch)) {
                        coralScoreGoalPose = Settings.coralLeftEF;
                        scoreCoralOnLevel();
                    }                
                }

                if (operatorChosenSideOfReef == SideOfReef.GH) {
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_RightBranch)) {
                        coralScoreGoalPose = Settings.coralRightGH;
                        scoreCoralOnLevel();
                    } else if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_LeftBranch)) {
                        coralScoreGoalPose = Settings.coralLeftGH;
                        scoreCoralOnLevel();
                    }                
                }

                if (operatorChosenSideOfReef == SideOfReef.IJ) {
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_RightBranch)) {
                        coralScoreGoalPose = Settings.coralRightIJ;
                        scoreCoralOnLevel();
                    } else if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_LeftBranch)) {
                        coralScoreGoalPose = Settings.coralLeftIJ;
                        scoreCoralOnLevel();
                    }                
                }

                if (operatorChosenSideOfReef == SideOfReef.KL) {
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_RightBranch)) {
                        coralScoreGoalPose = Settings.coralRightKL;
                        scoreCoralOnLevel();
                    } else if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_LeftBranch)) {
                        coralScoreGoalPose = Settings.coralLeftKL;
                        scoreCoralOnLevel();
                    }                
                }

                break;
            default:
                break;
        }
    }

    public void scoreCoralOnLevel() {
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral1)) {
            scoreCoralGoalLevel = ElevatorStates.L1;
            if (scoreCoralGoalLevel == ElevatorStates.L1 && isInReefZone == true) {
                robotState = RobotStates.coralScore1;
            } 
        } 
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral2)) {
            scoreCoralGoalLevel = ElevatorStates.L2;
            if (scoreCoralGoalLevel == ElevatorStates.L2 && isInReefZone == true) {
                robotState = RobotStates.coralScore2;
            } 
        } 
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral3)) {
            scoreCoralGoalLevel = ElevatorStates.L3; 
            if (scoreCoralGoalLevel == ElevatorStates.L3 && isInReefZone == true) {
                robotState = RobotStates.coralScore3;
            } 
        }  
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral4)) {
            scoreCoralGoalLevel = ElevatorStates.L4; 
            if (scoreCoralGoalLevel == ElevatorStates.L4 && isInReefZone == true) {
                robotState = RobotStates.coralScore4;
            } 
        } 
    }
}
