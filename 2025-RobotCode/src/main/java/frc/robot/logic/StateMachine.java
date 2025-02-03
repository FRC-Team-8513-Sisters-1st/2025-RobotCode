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

    ElevatorStates scoreCoralGoalLevel = ElevatorStates.stowed;
    boolean isInReefZone = false;
    FeederStation feederCloseOrFar = FeederStation.Close;
    public Pose2d goalFeederStation = new Pose2d();
    public Pose2d goalProcessor = new Pose2d();
    boolean climberButtonPressed = false;
    SideOfReef operatorChosenSideOfReef = SideOfReef.AB;
    String lastButtonChosen = "coral";

    public StateMachine(Robot thisRobotIn) {

        thisRobot = thisRobotIn;
    }

    public void enumRobotState() {

        switch (robotState) {
            case driving:
                //set all subsystem states
                algaeGroundStates = AlgaeGroundStates.stowed;
                algaeIntakeStates = AlgaeIntakeStates.stationary;
                coralIntakeStates = CoralIntakeStates.stationary;
                elevatorStates = ElevatorStates.stowed;
                drivebaseStates = DrivebaseStates.stowedLimits;

                if (thisRobot.coralReady2Score == true && lastButtonChosen == "coral") {
                    scoreCoralOnLevel();
                }
                if (thisRobot.algaeReady2Score== true && lastButtonChosen == "algae") {
                    takeAlgaeOnLevel();
                }
                
                if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_Algae2)) {
                    thisRobot.teleopController.operatorGoalAlgaeReefLevel = RobotStates.algaeIntakeL2;
                    lastButtonChosen = "algae";
                }
                if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_Algae3)) {
                    thisRobot.teleopController.operatorGoalAlgaeReefLevel = RobotStates.algaeIntakeL3;
                    lastButtonChosen = "algae";
                }

                if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_processor)) {
                    goalProcessor = Settings.processor;
                }

                climberButtonPressed = thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_Climb);
                if (climberStates == ClimberStates.stowed && climberButtonPressed){
                    climberStates = ClimberStates.armOut;
                    climberButtonPressed = false;
                }
                if (climberStates == ClimberStates.armOut && climberButtonPressed){
                    climberStates = ClimberStates.climbing;
                    climberButtonPressed = false;
                }
                if (climberStates == ClimberStates.climbing && climberButtonPressed){
                    climberStates = ClimberStates.stowed;
                    climberButtonPressed = false;
                }

                copilotSideOfReef();

                copilotLevelToScore();

                copilotCloseOrFar();

                forceCoralAndAlgae();

                operatorFeederStation();


                break;

            case algaeIntakeL3:
            climberStates = ClimberStates.stowed;
            algaeGroundStates = AlgaeGroundStates.stowed;
            algaeIntakeStates = AlgaeIntakeStates.intake;
            coralIntakeStates = CoralIntakeStates.stationary;                
            elevatorStates = ElevatorStates.L4;
            drivebaseStates = DrivebaseStates.L4Limits;

            if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_Drive)) {
                robotState = RobotStates.driving;
            }

            if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_processor)) {
                goalProcessor = Settings.processor;
            }

            if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_Algae2)) {
                robotState = RobotStates.algaeIntakeL2;
                lastButtonChosen = "algae";

            }

            copilotCloseOrFar();

            operatorFeederStation();

            scoreCoralOnLevelFromLevel();

            forceCoralAndAlgae();


            case algaeIntakeL2:
            climberStates = ClimberStates.stowed;
            algaeGroundStates = AlgaeGroundStates.stowed;
            algaeIntakeStates = AlgaeIntakeStates.intake;
            coralIntakeStates = CoralIntakeStates.stationary;                
            elevatorStates = ElevatorStates.L3;
            drivebaseStates = DrivebaseStates.L3Limits;

            if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_Drive)) {
                robotState = RobotStates.driving;

            }

            if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_processor)) {
                robotState = RobotStates.driving;
                goalProcessor = Settings.processor;
            }

            if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_Algae3)) {
                robotState = RobotStates.algaeIntakeL3;
                lastButtonChosen = "algae";

            }

            copilotCloseOrFar();

            operatorFeederStation();
            
            scoreCoralOnLevelFromLevel();

            forceCoralAndAlgae();

            case coralScore1:
            climberStates = ClimberStates.stowed;
            algaeGroundStates = AlgaeGroundStates.stowed;
            algaeIntakeStates = AlgaeIntakeStates.stationary;
            coralIntakeStates = CoralIntakeStates.outake;                
            elevatorStates = ElevatorStates.L1;
            drivebaseStates = DrivebaseStates.L1Limits;

            if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_Drive)) {
                robotState = RobotStates.driving;

            }

            if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_processor)) {
                robotState = RobotStates.driving;
                goalProcessor = Settings.processor;

            }


            copilotCloseOrFar();

            operatorFeederStation();
            
            scoreCoralOnLevelFromLevel();

            forceCoralAndAlgae();


            case coralScore2:
            climberStates = ClimberStates.stowed;
            algaeGroundStates = AlgaeGroundStates.stowed;
            algaeIntakeStates = AlgaeIntakeStates.stationary;
            coralIntakeStates = CoralIntakeStates.outake;                
            elevatorStates = ElevatorStates.L2;
            drivebaseStates = DrivebaseStates.L2Limits;

            if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_Drive)) {
                robotState = RobotStates.driving;

            }

            if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_processor)) {
                robotState = RobotStates.driving;
                goalProcessor = Settings.processor;

            }


            copilotCloseOrFar();

            operatorFeederStation();
            
            scoreCoralOnLevelFromLevel();

            forceCoralAndAlgae();


            case coralScore3:
            climberStates = ClimberStates.stowed;
            algaeGroundStates = AlgaeGroundStates.stowed;
            algaeIntakeStates = AlgaeIntakeStates.stationary;
            coralIntakeStates = CoralIntakeStates.outake;                
            elevatorStates = ElevatorStates.L3;
            drivebaseStates = DrivebaseStates.L3Limits;

            if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_Drive)) {
                robotState = RobotStates.driving;

            }

            if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_processor)) {
                robotState = RobotStates.driving;
                goalProcessor = Settings.processor;

            }


            copilotCloseOrFar();

            operatorFeederStation();
            
            scoreCoralOnLevelFromLevel();
            
            forceCoralAndAlgae();


            case coralScore4:
            climberStates = ClimberStates.stowed;
            algaeGroundStates = AlgaeGroundStates.stowed;
            algaeIntakeStates = AlgaeIntakeStates.stationary;
            coralIntakeStates = CoralIntakeStates.outake;                
            elevatorStates = ElevatorStates.L4;
            drivebaseStates = DrivebaseStates.L4Limits;

            if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_Drive)) {
                robotState = RobotStates.driving;

            }

            if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_processor)) {
                robotState = RobotStates.driving;
                goalProcessor = Settings.processor;

            }


            copilotCloseOrFar();

            operatorFeederStation();
            
            scoreCoralOnLevelFromLevel();

            forceCoralAndAlgae();


            case coral1:
            climberStates = ClimberStates.stowed;
            algaeGroundStates = AlgaeGroundStates.stowed;
            algaeIntakeStates = AlgaeIntakeStates.stationary;
            coralIntakeStates = CoralIntakeStates.stationary;                
            elevatorStates = ElevatorStates.L1;
            drivebaseStates = DrivebaseStates.L1Limits;

            if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_Drive)) {
                robotState = RobotStates.driving;

            }
            
            scoreCoralOnLevelFromLevel();

            forceCoralAndAlgae();


            case coral2:
            climberStates = ClimberStates.stowed;
            algaeGroundStates = AlgaeGroundStates.stowed;
            algaeIntakeStates = AlgaeIntakeStates.stationary;
            coralIntakeStates = CoralIntakeStates.stationary;                
            elevatorStates = ElevatorStates.L2;
            drivebaseStates = DrivebaseStates.L2Limits;

            if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_Drive)) {
                robotState = RobotStates.driving;

            }
            
            scoreCoralOnLevelFromLevel();

            forceCoralAndAlgae();


            case coral3:
            climberStates = ClimberStates.stowed;
            algaeGroundStates = AlgaeGroundStates.stowed;
            algaeIntakeStates = AlgaeIntakeStates.stationary;
            coralIntakeStates = CoralIntakeStates.stationary;                
            elevatorStates = ElevatorStates.L3;
            drivebaseStates = DrivebaseStates.L3Limits;

            if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_Drive)) {
                robotState = RobotStates.driving;

            }
            
            scoreCoralOnLevelFromLevel();

            forceCoralAndAlgae();


            case coral4:
            climberStates = ClimberStates.stowed;
            algaeGroundStates = AlgaeGroundStates.stowed;
            algaeIntakeStates = AlgaeIntakeStates.stationary;
            coralIntakeStates = CoralIntakeStates.stationary;                
            elevatorStates = ElevatorStates.L4;
            drivebaseStates = DrivebaseStates.L4Limits;

            if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_Drive)) {
                robotState = RobotStates.driving;

            }
  
            scoreCoralOnLevelFromLevel();

            forceCoralAndAlgae();


            default:
                break;
        }
    }

    // scores coral on copilot selected level 
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

    // score coral on any level from any level bypassing copilot selecting side and level
    public void scoreCoralOnLevelFromLevel() {
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral1)) {
            robotState = RobotStates.coralScore1;

        }
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral2)) {
            robotState = RobotStates.coralScore2;

        }
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral3)) {
            robotState = RobotStates.coralScore3;

        }
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral4)) {
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

    // store if operator hits side of reef button
    public void copilotSideOfReef () {
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
    }
    
    // store which level operator chooses
    public void copilotLevelToScore () {
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral1)) {
            scoreCoralGoalLevel = ElevatorStates.L1; 
            lastButtonChosen = "coral";
        } 
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral2)) {
            scoreCoralGoalLevel = ElevatorStates.L2;
            lastButtonChosen = "coral";

        } 
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral3)) {
            scoreCoralGoalLevel = ElevatorStates.L3; 
            lastButtonChosen = "coral";

        }  
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral4)) {
            scoreCoralGoalLevel = ElevatorStates.L4;
            lastButtonChosen = "coral";
 
        }
    }

    // store close or far for feeding station
    public void copilotCloseOrFar () {
        if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_Far)) {
            feederCloseOrFar = FeederStation.Far; 

        } 
        if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_Close)) {
            feederCloseOrFar = FeederStation.Close; 

        }
    }

    // force intake and outake coral and algae
    public void forceCoralAndAlgae() {
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_AlgaeIntake)) {
            algaeIntakeStates = AlgaeIntakeStates.intake;

        } 
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_AlgaeOutake)) {
            algaeIntakeStates = AlgaeIntakeStates.outake;

        }
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_CoralIntake)) {
            coralIntakeStates = CoralIntakeStates.intake;

        } 
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_CoralOutake)) {
            coralIntakeStates = CoralIntakeStates.outake;

        }
    }

    // go to feeder station and intake coral
    public void operatorFeederStation() {
        if (feederCloseOrFar == FeederStation.Far) {
            if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_LeftFeederSt)) {
                goalFeederStation = Settings.rightFarFeederStation;
                coralIntakeStates = CoralIntakeStates.intake;

            } else if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_RightFeederSt)) {
                goalFeederStation = Settings.leftFarFeederStation;
                coralIntakeStates = CoralIntakeStates.intake;

            }
        }
        if (feederCloseOrFar == FeederStation.Close) {
            if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_LeftFeederSt)) {
                goalFeederStation = Settings.rightCloseFeederStation;
                coralIntakeStates = CoralIntakeStates.intake;

            } else if (thisRobot.teleopController.driverXboxController.getRawButtonPressed(Settings.buttonId_RightFeederSt)) {
                goalFeederStation = Settings.leftCloseFeederStation;
                coralIntakeStates = CoralIntakeStates.intake;

            }
        }
    }
    public boolean isRobotInReefZone() {
        double x = thisRobot.drivebase.swerveDrive.getPose().minus(Settings.reefZone).getX();
        double y = thisRobot.drivebase.swerveDrive.getPose().minus(Settings.reefZone).getY();
        double distance = Math.sqrt(x*x+y*y);
        return distance < Settings.minDistanceFromReefZoneMeter;
    }
}
