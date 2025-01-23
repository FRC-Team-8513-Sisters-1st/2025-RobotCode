package frc.robot.logic;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.logic.Enums.AlgaeGroundStates;
import frc.robot.logic.Enums.AlgaeIntakeStates;
import frc.robot.logic.Enums.ClimberStates;
import frc.robot.logic.Enums.CoralIntakeStates;
import frc.robot.logic.Enums.DrivebaseStates;
import frc.robot.logic.Enums.ElevatorStates;
import frc.robot.logic.Enums.FeederStation;
import frc.robot.logic.Enums.RobotStates;
import frc.robot.logic.Enums.SideOfReef;

public class StateMachine {

    Robot thisRobot;

    // robot states
    RobotStates robotState = RobotStates.driving;
    ClimberStates climberStates = ClimberStates.stowed;
    AlgaeGroundStates algaeGroundStates = AlgaeGroundStates.stowed;
    AlgaeIntakeStates algaeIntakeStates = AlgaeIntakeStates.stationary;
    CoralIntakeStates coralIntakeStates = CoralIntakeStates.stationary;
    ElevatorStates elevatorStates = ElevatorStates.stowed;
    DrivebaseStates drivebaseStates = DrivebaseStates.stowedLimits;

    Pose2d coralScoreGoalPose;
    SideOfReef operatorChosenSideOfReef;
    ElevatorStates scoreCoralGoalLevel;
    boolean isInReefZone = false;
    FeederStation feederCloseOrFar;
    Pose2d goalFeederStation;
    Pose2d goalProcessor;

    public StateMachine(Robot thisRobotIn) {

        thisRobot = thisRobotIn;
    }

    public void enumRobotState() {

        switch (robotState) {
            case driving:
                //set all subsystem states
                climberStates = ClimberStates.stowed;
                algaeGroundStates = AlgaeGroundStates.stowed;
                algaeIntakeStates = AlgaeIntakeStates.stationary;
                coralIntakeStates = CoralIntakeStates.stationary;
                elevatorStates = ElevatorStates.stowed;
                drivebaseStates = DrivebaseStates.stowedLimits;

                if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_CoralIntake)) {
                    robotState = RobotStates.coralIntakeFeederSt;
                }

                if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_processor)) {
                    goalProcessor = Settings.processor;
                }
                
                if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_Climb)) {
                    robotState = RobotStates.preClimb;
                    if (robotState == RobotStates.preClimb && thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_Climb)){
                        robotState = RobotStates.climbDeep;
                    }
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

                // store which level operator choses
                if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral1)) {
                    scoreCoralGoalLevel = ElevatorStates.L1; 
                } 
                if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral2)) {
                    scoreCoralGoalLevel = ElevatorStates.L2;
                } 
                if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral3)) {
                    scoreCoralGoalLevel = ElevatorStates.L3; 
                }  
                if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral4)) {
                    scoreCoralGoalLevel = ElevatorStates.L4; 
                }

                // store close or far for feeding station
                if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_Far)) {
                    feederCloseOrFar = FeederStation.Far; 
                } 
                if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Close)) {
                    feederCloseOrFar = FeederStation.Close; 
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

                // take algae
                if (operatorChosenSideOfReef == SideOfReef.AB) {
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_Algae2)) {
                        coralScoreGoalPose = Settings.coralRightAB;
                        takeAlgaeOnLevel();
                    } else if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_Algae3)) {
                        coralScoreGoalPose = Settings.coralRightAB;
                        takeAlgaeOnLevel();
                    }
                }
                if (operatorChosenSideOfReef == SideOfReef.CD) {
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_Algae2)) {
                        coralScoreGoalPose = Settings.coralRightCD;
                        takeAlgaeOnLevel();
                    } else if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_Algae3)) {
                        coralScoreGoalPose = Settings.coralRightCD;
                        takeAlgaeOnLevel();
                    }
                }
                if (operatorChosenSideOfReef == SideOfReef.EF) {
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_Algae2)) {
                        coralScoreGoalPose = Settings.coralRightEF;
                        takeAlgaeOnLevel();
                    } else if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_Algae3)) {
                        coralScoreGoalPose = Settings.coralRightEF;
                        takeAlgaeOnLevel();
                    }
                }
                if (operatorChosenSideOfReef == SideOfReef.GH) {
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_Algae2)) {
                        coralScoreGoalPose = Settings.coralRightGH;
                        takeAlgaeOnLevel();
                    } else if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_Algae3)) {
                        coralScoreGoalPose = Settings.coralRightGH;
                        takeAlgaeOnLevel();
                    }
                }
                if (operatorChosenSideOfReef == SideOfReef.IJ) {
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_Algae2)) {
                        coralScoreGoalPose = Settings.coralRightIJ;
                        takeAlgaeOnLevel();
                    } else if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_Algae3)) {
                        coralScoreGoalPose = Settings.coralRightIJ;
                        takeAlgaeOnLevel();
                    }
                }
                if (operatorChosenSideOfReef == SideOfReef.KL) {
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_Algae2)) {
                        coralScoreGoalPose = Settings.coralRightKL;
                        takeAlgaeOnLevel();
                    } else if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_Algae3)) {
                        coralScoreGoalPose = Settings.coralRightKL;
                        takeAlgaeOnLevel();
                    }
                }

                // go to feeder station and intake coral
                if (feederCloseOrFar == FeederStation.Far) {
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_RightFeederSt)) {
                        goalFeederStation = Settings.rightFarFeederStation;
                        coralIntakeStates = CoralIntakeStates.intake;
                    } else if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_LeftFeederSt)) {
                        goalFeederStation = Settings.leftFarFeederStation;
                        coralIntakeStates = CoralIntakeStates.intake;
                    }
                }
                if (feederCloseOrFar == FeederStation.Close) {
                    if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_RightFeederSt)) {
                        goalFeederStation = Settings.rightCloseFeederStation;
                        coralIntakeStates = CoralIntakeStates.intake;
                    } else if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_LeftFeederSt)) {
                        goalFeederStation = Settings.leftCloseFeederStation;
                        coralIntakeStates = CoralIntakeStates.intake;
                    }
                }


                break;
            default:
                break;
        }
    }

    // scores coral on selected level 
    public void scoreCoralOnLevel() { 
        if (scoreCoralGoalLevel == ElevatorStates.L1 && isInReefZone == true) {
            robotState = RobotStates.coralScore1;
        }
        if (scoreCoralGoalLevel == ElevatorStates.L2 && isInReefZone == true) {
            robotState = RobotStates.coralScore2;
        }
        if (scoreCoralGoalLevel == ElevatorStates.L3 && isInReefZone == true) {
            robotState = RobotStates.coralScore3;
        }
        if (scoreCoralGoalLevel == ElevatorStates.L4 && isInReefZone == true) {
            robotState = RobotStates.coralScore4;
        }
    }

    // takes algae from selected level
    public void takeAlgaeOnLevel() {
        if (scoreCoralGoalLevel == ElevatorStates.L3 && isInReefZone == true) {
            robotState = RobotStates.algaeIntakeL2;
        }
        if (scoreCoralGoalLevel == ElevatorStates.L4 && isInReefZone == true) {
            robotState = RobotStates.algaeIntakeL3;
        }
    }
}
