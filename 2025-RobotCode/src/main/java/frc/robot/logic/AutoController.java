package frc.robot.logic;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.logic.Enums.AutoRoutines;
import frc.robot.logic.Enums.CoralIntakeStates;
import frc.robot.logic.Enums.ElevatorStates;

public class AutoController {

    Robot thisRobot;
    public AutoRoutines autoRoutine = AutoRoutines.DoNothing;
    public int autoStep = 0;
    public boolean firstAutoBeingRun = true;

    public SendableChooser<String> autoSelector;

    double timeStepStarted = 0;

    // custom auto
    Pose2d customAutoStartPose = Settings.autoProcessorStartPose;
    Pose2d[] customAutoPoses = { Settings.coralLeftEF, Settings.rightCloseFeederStationAP, Settings.coralRightEF,
            Settings.rightCloseFeederStationAP, Settings.coralLeftKL };
    ElevatorStates[] customElevatorStates = { ElevatorStates.L4, ElevatorStates.L1, ElevatorStates.L4,
            ElevatorStates.L1, ElevatorStates.L4 };

    public AutoController(Robot thisRobotIn) {
        thisRobot = thisRobotIn;

        // create auto selector with each enum option
        autoSelector = new SendableChooser<>();
        autoSelector.setDefaultOption(AutoRoutines.values()[0].toString(), AutoRoutines.values()[0].toString());
        for (int i = 1; i < AutoRoutines.values().length; i++) {
            if (AutoRoutines.values()[i].toString().charAt(0) != '_') {
                autoSelector.addOption(AutoRoutines.values()[i].toString(), AutoRoutines.values()[i].toString());
            }

        }
        SmartDashboard.putData("Auton Selector", autoSelector);

    }

    public void autoInit() {
        updateAutoRoutine();
        autoStep = 0;
    }

    public void autoDis() {
        updateAutoRoutine();
    }

    public void updateAutoRoutine(){
        try {
            autoRoutine = AutoRoutines.valueOf(autoSelector.getSelected());
        } catch (Exception e) {
            autoRoutine = AutoRoutines.DoNothing;
        }
    }

    public void autoPeriodic() {
        switch (autoRoutine) {
            case DoNothing:
                thisRobot.drivebase.swerveDrive.lockPose();
                break;
            case mid_GH3R:
                switch (autoStep) {
                    case 0:
                        if (firstAutoBeingRun) {
                            firstAutoBeingRun = false;
                            if (Robot.isSimulation()) {
                                if (thisRobot.onRedAlliance) {
                                    thisRobot.drivebase.swerveDrive.resetOdometry(
                                            thisRobot.drivebase.flipPoseToRed(Settings.autoMidStartPose));
                                } else {
                                    thisRobot.drivebase.swerveDrive.resetOdometry(Settings.autoMidStartPose);
                                }
                            }
                            thisRobot.drivebase.resetAPPIDControllers(Settings.coralRightGH);
                        }

                        if (thisRobot.drivebase.attackPoint(Settings.coralRightGH, 2)) {
                            autoStep = 10;
                        }
                        thisRobot.elevator.state = ElevatorStates.L3;
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.state = CoralIntakeStates.stationary;
                        thisRobot.coral.setMotorPower();
                        break;
                    case 10:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.state = CoralIntakeStates.outake;
                        thisRobot.coral.setMotorPower();
                        thisRobot.drivebase.swerveDrive.lockPose();
                        break;
                    default:
                        break;
                }
                break;
            case processor_EF2L:
                switch (autoStep) {
                    case 0:
                        if (firstAutoBeingRun) {
                            firstAutoBeingRun = false;
                            if (Robot.isSimulation()) {
                                if (thisRobot.onRedAlliance) {
                                    thisRobot.drivebase.swerveDrive.resetOdometry(
                                            thisRobot.drivebase.flipPoseToRed(Settings.autoProcessorStartPose));
                                } else {
                                    thisRobot.drivebase.swerveDrive.resetOdometry(Settings.autoProcessorStartPose);
                                }
                            }
                            thisRobot.drivebase.resetAPPIDControllers(Settings.coralLeftEF);
                        }

                        if (thisRobot.drivebase.attackPoint(Settings.coralLeftEF, 2)) {
                            autoStep = 10;
                        }
                        thisRobot.elevator.state = ElevatorStates.L2;
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.state = CoralIntakeStates.stationary;
                        thisRobot.coral.setMotorPower();
                        break;
                    case 10:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.state = CoralIntakeStates.outake;
                        thisRobot.coral.setMotorPower();
                        thisRobot.drivebase.swerveDrive.lockPose();
                        break;
                    default:
                        break;
                }
                break;
            case processor_EF2L_RCFS_AB2L_RCFS_AB2R:
                switch (autoStep) {
                    case 0:
                        firstAutoBeingRun = false;
                        if (Robot.isSimulation()) {
                            if (thisRobot.onRedAlliance) {
                                thisRobot.drivebase.swerveDrive.resetOdometry(
                                        thisRobot.drivebase.flipPoseToRed(Settings.autoProcessorStartPose));
                            } else {
                                thisRobot.drivebase.swerveDrive
                                        .resetOdometry(Settings.autoProcessorStartPose);
                            }
                        }
                        thisRobot.drivebase.initAstarAndAP(
                                Settings.coralLeftEF.transformBy(Settings.astarReefPoseOffset), Settings.coralLeftEF);
                        thisRobot.coral.state = CoralIntakeStates.stationary;
                        thisRobot.elevator.state = ElevatorStates.L2;
                        autoStep = 5;
                    case 5:
                        if (thisRobot.drivebase.fromOTFSwitchToAP()) {
                            autoStep = 10;
                            thisRobot.coral.state = CoralIntakeStates.outake;
                            timeStepStarted = Timer.getFPGATimestamp();
                        }
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        break;
                    case 10:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        thisRobot.drivebase.swerveDrive.lockPose();
                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.5) {
                            thisRobot.coral.setMotorPower();
                            thisRobot.drivebase.initAstarAndAP(
                                    Settings.rightCloseFeederStationAP.transformBy(Settings.astarFeederStPoseOffset),
                                    Settings.rightCloseFeederStationAP);
                            autoStep = 20;
                            thisRobot.coral.state = CoralIntakeStates.outake;
                            timeStepStarted = Timer.getFPGATimestamp();
                        }
                        break;
                    case 20:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.25) {
                            thisRobot.elevator.state = ElevatorStates.L1;
                        }
                        if (thisRobot.drivebase.fromOTFSwitchToAP()) {
                            autoStep = 25;
                            thisRobot.elevator.state = ElevatorStates.L2;
                            thisRobot.drivebase.initAstarAndAP(
                                    Settings.coralLeftAB.transformBy(Settings.astarReefPoseOffset),
                                    Settings.coralLeftAB);
                        }
                        break;
                    case 25:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        if (thisRobot.drivebase.fromOTFSwitchToAP()) {
                            autoStep = 30;
                            timeStepStarted = Timer.getFPGATimestamp();
                            thisRobot.coral.state = CoralIntakeStates.outake;
                        }
                        break;
                    case 30:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.drivebase.swerveDrive.lockPose();
                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.5) {
                            thisRobot.coral.setMotorPower();
                            thisRobot.drivebase.initAstarAndAP(
                                    Settings.rightCloseFeederStationAP.transformBy(Settings.astarFeederStPoseOffset),
                                    Settings.rightCloseFeederStationAP);
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 35;
                        }
                        break;
                    case 35:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.25) {
                            thisRobot.elevator.state = ElevatorStates.L1;
                        }
                        if (thisRobot.drivebase.fromOTFSwitchToAP()) {
                            autoStep = 40;
                            thisRobot.elevator.state = ElevatorStates.L2;
                            thisRobot.drivebase.initAstarAndAP(
                                    Settings.coralRightAB.transformBy(Settings.astarReefPoseOffset),
                                    Settings.coralRightAB);
                        }
                        break;
                    case 40:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        if (thisRobot.drivebase.fromOTFSwitchToAP()) {
                            autoStep = 45;
                            thisRobot.coral.state = CoralIntakeStates.outake;
                        }
                        break;
                    case 45:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        thisRobot.drivebase.swerveDrive.lockPose();
                        break;
                    default:
                        break;
                }
                break;
            case far_KL3L_RFFS_CD3L_RFFS_CD4R:
                switch (autoStep) {
                    case 0:
                        firstAutoBeingRun = false;
                        if (Robot.isSimulation()) {
                            if (thisRobot.onRedAlliance) {
                                thisRobot.drivebase.swerveDrive.resetOdometry(
                                        thisRobot.drivebase.flipPoseToRed(Settings.autoFarStartPose));
                            } else {
                                thisRobot.drivebase.swerveDrive
                                        .resetOdometry(Settings.autoFarStartPose);
                            }
                        }
                        thisRobot.drivebase.initAstarAndAP(
                                Settings.coralLeftKL.transformBy(Settings.astarReefPoseOffset), Settings.coralLeftKL);
                        thisRobot.coral.state = CoralIntakeStates.stationary;
                        thisRobot.elevator.state = ElevatorStates.L3;
                        autoStep = 5;
                    case 5:
                        if (thisRobot.drivebase.fromOTFSwitchToAP()) {
                            autoStep = 10;
                            thisRobot.coral.state = CoralIntakeStates.outake;
                            timeStepStarted = Timer.getFPGATimestamp();
                        }
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        break;
                    case 10:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        thisRobot.drivebase.swerveDrive.lockPose();
                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.5) {
                            thisRobot.coral.setMotorPower();
                            thisRobot.drivebase.initAstarAndAP(
                                    Settings.rightFarFeederStationAP.transformBy(Settings.astarFeederStPoseOffset),
                                    Settings.rightFarFeederStationAP);
                            autoStep = 20;
                            thisRobot.coral.state = CoralIntakeStates.outake;
                            timeStepStarted = Timer.getFPGATimestamp();
                        }
                        break;
                    case 20:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.25) {
                            thisRobot.elevator.state = ElevatorStates.L1;
                        }
                        if (thisRobot.drivebase.fromOTFSwitchToAP()) {
                            autoStep = 25;
                            thisRobot.elevator.state = ElevatorStates.L3;
                            thisRobot.drivebase.initAstarAndAP(
                                    Settings.coralLeftCD.transformBy(Settings.astarReefPoseOffset),
                                    Settings.coralLeftCD);
                        }
                        break;
                    case 25:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        if (thisRobot.drivebase.fromOTFSwitchToAP()) {
                            autoStep = 30;
                            timeStepStarted = Timer.getFPGATimestamp();
                            thisRobot.coral.state = CoralIntakeStates.outake;
                        }
                        break;
                    case 30:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.drivebase.swerveDrive.lockPose();
                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.5) {
                            thisRobot.coral.setMotorPower();
                            thisRobot.drivebase.initAstarAndAP(
                                    Settings.rightFarFeederStationAP.transformBy(Settings.astarFeederStPoseOffset),
                                    Settings.rightFarFeederStationAP);
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 35;
                        }
                        break;
                    case 35:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.25) {
                            thisRobot.elevator.state = ElevatorStates.L1;
                        }
                        if (thisRobot.drivebase.fromOTFSwitchToAP()) {
                            autoStep = 40;
                            thisRobot.elevator.state = ElevatorStates.L4;
                            thisRobot.drivebase.initAstarAndAP(
                                    Settings.coralRightCD.transformBy(Settings.astarReefPoseOffset),
                                    Settings.coralRightCD);
                        }
                        break;
                    case 40:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        if (thisRobot.drivebase.fromOTFSwitchToAP()) {
                            autoStep = 45;
                            thisRobot.coral.state = CoralIntakeStates.outake;
                        }
                        break;
                    case 45:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        thisRobot.drivebase.swerveDrive.lockPose();
                        break;
                    default:
                        break;
                }
                break;
            case mid_EF4R_RCFS_CD4R_RCFS_CD4L:
                switch (autoStep) {
                    case 0:
                        firstAutoBeingRun = false;
                        if (Robot.isSimulation()) {
                            if (thisRobot.onRedAlliance) {
                                thisRobot.drivebase.swerveDrive.resetOdometry(
                                        thisRobot.drivebase.flipPoseToRed(Settings.autoMidStartPose));
                            } else {
                                thisRobot.drivebase.swerveDrive
                                        .resetOdometry(Settings.autoMidStartPose);
                            }
                        }
                        thisRobot.drivebase.initAstarAndAP(
                                Settings.coralRightEF.transformBy(Settings.astarReefPoseOffset), Settings.coralRightEF);
                        thisRobot.coral.state = CoralIntakeStates.stationary;
                        thisRobot.elevator.state = ElevatorStates.L4;
                        autoStep = 5;
                    case 5:
                        if (thisRobot.drivebase.fromOTFSwitchToAP()) {
                            autoStep = 10;
                            thisRobot.coral.state = CoralIntakeStates.outake;
                            timeStepStarted = Timer.getFPGATimestamp();
                        }
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        break;
                    case 10:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        thisRobot.drivebase.swerveDrive.lockPose();
                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.5) {
                            thisRobot.coral.setMotorPower();
                            thisRobot.drivebase.initAstarAndAP(
                                    Settings.rightCloseFeederStationAP.transformBy(Settings.astarFeederStPoseOffset),
                                    Settings.rightCloseFeederStationAP);
                            autoStep = 20;
                            thisRobot.coral.state = CoralIntakeStates.outake;
                            timeStepStarted = Timer.getFPGATimestamp();
                        }
                        break;
                    case 20:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.25) {
                            thisRobot.elevator.state = ElevatorStates.L1;
                        }
                        if (thisRobot.drivebase.fromOTFSwitchToAP()) {
                            autoStep = 25;
                            thisRobot.elevator.state = ElevatorStates.L4;
                            thisRobot.drivebase.initAstarAndAP(
                                    Settings.coralRightCD.transformBy(Settings.astarReefPoseOffset),
                                    Settings.coralRightCD);
                        }
                        break;
                    case 25:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        if (thisRobot.drivebase.fromOTFSwitchToAP()) {
                            autoStep = 30;
                            timeStepStarted = Timer.getFPGATimestamp();
                            thisRobot.coral.state = CoralIntakeStates.outake;
                        }
                        break;
                    case 30:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.drivebase.swerveDrive.lockPose();
                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.5) {
                            thisRobot.coral.setMotorPower();
                            thisRobot.drivebase.initAstarAndAP(
                                    Settings.rightCloseFeederStationAP.transformBy(Settings.astarFeederStPoseOffset),
                                    Settings.rightCloseFeederStationAP);
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 35;
                        }
                        break;
                    case 35:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.25) {
                            thisRobot.elevator.state = ElevatorStates.L1;
                        }
                        if (thisRobot.drivebase.fromOTFSwitchToAP()) {
                            autoStep = 40;
                            thisRobot.elevator.state = ElevatorStates.L4;
                            thisRobot.drivebase.initAstarAndAP(
                                    Settings.coralLeftCD.transformBy(Settings.astarReefPoseOffset),
                                    Settings.coralLeftCD);
                        }
                        break;
                    case 40:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        if (thisRobot.drivebase.fromOTFSwitchToAP()) {
                            autoStep = 45;
                            thisRobot.coral.state = CoralIntakeStates.outake;
                        }
                        break;
                    case 45:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        thisRobot.drivebase.swerveDrive.lockPose();
                        break;
                    default:
                        break;
                }
                break;
            case customAuto:
                switch (autoStep) {
                    case 0:

                        firstAutoBeingRun = false;
                        if (Robot.isSimulation()) {
                            if (thisRobot.onRedAlliance) {
                                thisRobot.drivebase.swerveDrive.resetOdometry(
                                        thisRobot.drivebase.flipPoseToRed(customAutoStartPose));
                            } else {
                                thisRobot.drivebase.swerveDrive.resetOdometry(customAutoStartPose);
                            }
                        }
                        thisRobot.drivebase.initPathToPoint(customAutoPoses[0]);
                        thisRobot.coral.state = CoralIntakeStates.stationary;
                        thisRobot.elevator.state = customElevatorStates[0];
                        autoStep = 5;
                        // intentially no break
                    case 5:
                        if (thisRobot.drivebase.followOTFPath()) {
                            autoStep = 10;
                            thisRobot.coral.state = CoralIntakeStates.outake;
                            timeStepStarted = Timer.getFPGATimestamp();
                        }

                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        break;

                    case 10:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        thisRobot.drivebase.swerveDrive.lockPose();
                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.5) {
                            thisRobot.coral.setMotorPower();
                            thisRobot.drivebase.initPathToPoint(customAutoPoses[1]);
                            autoStep = 20;
                            thisRobot.elevator.state = customElevatorStates[1];
                            thisRobot.coral.state = CoralIntakeStates.outake;

                        }
                        break;

                    case 20:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();

                        if (thisRobot.drivebase.followOTFPath()) {
                            autoStep = 25;
                            thisRobot.elevator.state = customElevatorStates[2];
                            thisRobot.drivebase.initPathToPoint(customAutoPoses[2]);
                        }
                        break;
                    case 25:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        if (thisRobot.drivebase.followOTFPath()) {
                            autoStep = 30;
                            timeStepStarted = Timer.getFPGATimestamp();
                            thisRobot.coral.state = CoralIntakeStates.outake;
                        }
                        break;

                    case 30:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        thisRobot.drivebase.swerveDrive.lockPose();
                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.5) {
                            autoStep = 35;
                            thisRobot.elevator.state = customElevatorStates[3];
                            thisRobot.elevator.setMotorPower();
                            thisRobot.coral.state = CoralIntakeStates.outake;
                            thisRobot.drivebase.initPathToPoint(customAutoPoses[3]);
                        }
                        break;
                    case 35:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        if (thisRobot.drivebase.followOTFPath()) {
                            autoStep = 40;
                            thisRobot.elevator.state = customElevatorStates[4];
                            thisRobot.drivebase.initPathToPoint(customAutoPoses[4]);
                        }
                        break;

                    case 40:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        if (thisRobot.drivebase.followOTFPath()) {
                            autoStep = 45;
                            thisRobot.coral.state = CoralIntakeStates.outake;
                        }
                        break;

                    case 45:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        thisRobot.drivebase.swerveDrive.lockPose();
                        break;

                    default:
                        break;
                }
            default:
                break;
        }
    }

}
