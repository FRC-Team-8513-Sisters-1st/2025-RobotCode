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

    // auto variables
    Pose2d processor_EF4L_RCFS_EF4R_RCFS_KL4LStartPose = Settings.autoProcessorStartPose;
    Pose2d[] processor_EF4L_RCFS_EF4R_RCFS_KL4LAutoPoses = { Settings.coralLeftEF, Settings.rightCloseFeederStationAP,
            Settings.coralRightEF,
            Settings.rightCloseFeederStationAP, Settings.coralLeftKL };
    ElevatorStates[] processor_EF4L_RCFS_EF4R_RCFS_KL4LElevatorStates = { ElevatorStates.L4, ElevatorStates.L1,
            ElevatorStates.L4,
            ElevatorStates.L1, ElevatorStates.L4 };

    Pose2d far_KL3L_RFFS_CD3L_RFFS_AB2LStartPose = Settings.autoFarStartPose;
    Pose2d[] far_KL3L_RFFS_CD3L_RFFS_AB2LAutoPoses = { Settings.coralLeftKL, Settings.rightFarFeederStationAP,
            Settings.coralLeftCD, Settings.rightFarFeederStationAP, Settings.coralLeftAB };
    ElevatorStates[] far_KL3L_RFFS_CD3L_RFFS_AB2LElevatorStates = { ElevatorStates.L3, ElevatorStates.L1,
            ElevatorStates.L3,
            ElevatorStates.L1, ElevatorStates.L2 };

    Pose2d mid_GH4L_RCFS_EF4L_RCFS_CD4LStartPose = Settings.autoFarStartPose;
    Pose2d[] mid_GH4L_RCFS_EF4L_RCFS_CD4LAutoPoses = { Settings.coralLeftGH, Settings.rightCloseFeederStationAP,
            Settings.coralLeftEF, Settings.rightCloseFeederStationAP, Settings.coralLeftCD };
    ElevatorStates[] mid_GH4L_RCFS_EF4L_RCFS_CD4LElevatorStates = { ElevatorStates.L4, ElevatorStates.L1,
            ElevatorStates.L4,
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
        autoRoutine = AutoRoutines.valueOf(autoSelector.getSelected());
        autoStep = 0;
    }

    public void autoDis() {
        autoRoutine = AutoRoutines.valueOf(autoSelector.getSelected());
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
            case processor_EF4L_RCFS_EF4R_RCFS_KL4L:
                switch (autoStep) {
                    case 0:
                        firstAutoBeingRun = false;
                        if (Robot.isSimulation()) {
                            if (thisRobot.onRedAlliance) {
                                thisRobot.drivebase.swerveDrive.resetOdometry(
                                        thisRobot.drivebase.flipPoseToRed(processor_EF4L_RCFS_EF4R_RCFS_KL4LStartPose));
                            } else {
                                thisRobot.drivebase.swerveDrive
                                        .resetOdometry(processor_EF4L_RCFS_EF4R_RCFS_KL4LStartPose);
                            }
                        }
                        thisRobot.drivebase.initAstarAndAP(Settings.coralLeftEF,
                                Settings.coralLeftEF.transformBy(Settings.astarReefPoseOffset));
                        thisRobot.coral.state = CoralIntakeStates.stationary;
                        thisRobot.elevator.state = processor_EF4L_RCFS_EF4R_RCFS_KL4LElevatorStates[0];
                        autoStep = 5;
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
                            thisRobot.drivebase.initAstarAndAP(Settings.rightCloseFeederStationAP,
                                    Settings.rightCloseFeederStationAP.transformBy(Settings.astarFeederStPoseOffset));
                            autoStep = 20;
                            thisRobot.elevator.state = processor_EF4L_RCFS_EF4R_RCFS_KL4LElevatorStates[1];
                            thisRobot.coral.state = CoralIntakeStates.outake;
                        }
                        break;
                    case 20:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        if (thisRobot.drivebase.followOTFPath()) {
                            autoStep = 25;
                            thisRobot.elevator.state = processor_EF4L_RCFS_EF4R_RCFS_KL4LElevatorStates[2];
                            thisRobot.drivebase.initAstarAndAP(Settings.coralRightEF,
                                    Settings.coralRightEF.transformBy(Settings.astarReefPoseOffset));
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
                        thisRobot.drivebase.swerveDrive.lockPose();
                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.5) {
                            thisRobot.coral.setMotorPower();
                            thisRobot.drivebase.initAstarAndAP(Settings.rightFarFeederStationAP,
                            Settings.rightFarFeederStationAP.transformBy(Settings.astarFeederStPoseOffset));
                            autoStep = 20;
                            thisRobot.elevator.state = processor_EF4L_RCFS_EF4R_RCFS_KL4LElevatorStates[3];
                        }
                        break;
                    case 35:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        if (thisRobot.drivebase.followOTFPath()) {
                            autoStep = 40;
                            thisRobot.elevator.state = processor_EF4L_RCFS_EF4R_RCFS_KL4LElevatorStates[4];
                            thisRobot.drivebase.initAstarAndAP( Settings.coralLeftKL,
                            Settings.coralLeftKL.transformBy(Settings.astarReefPoseOffset));
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
            case far_KL3L_RFFS_CD3L_RFFS_AB2L:
                switch (autoStep) {
                    case 0:
                        firstAutoBeingRun = false;
                        if (Robot.isSimulation()) {
                            if (thisRobot.onRedAlliance) {
                                thisRobot.drivebase.swerveDrive.resetOdometry(
                                        thisRobot.drivebase.flipPoseToRed(far_KL3L_RFFS_CD3L_RFFS_AB2LStartPose));
                            } else {
                                thisRobot.drivebase.swerveDrive
                                        .resetOdometry(far_KL3L_RFFS_CD3L_RFFS_AB2LStartPose);
                            }
                        }
                        thisRobot.drivebase.initPathToPoint(far_KL3L_RFFS_CD3L_RFFS_AB2LAutoPoses[0]);
                        thisRobot.coral.state = CoralIntakeStates.stationary;
                        thisRobot.elevator.state = far_KL3L_RFFS_CD3L_RFFS_AB2LElevatorStates[0];
                        autoStep = 5;
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
                            thisRobot.drivebase.initPathToPoint(far_KL3L_RFFS_CD3L_RFFS_AB2LAutoPoses[1]);
                            autoStep = 20;
                            thisRobot.elevator.state = far_KL3L_RFFS_CD3L_RFFS_AB2LElevatorStates[1];
                            thisRobot.coral.state = CoralIntakeStates.outake;
                        }
                        break;
                    case 20:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        if (thisRobot.drivebase.followOTFPath()) {
                            autoStep = 25;
                            thisRobot.elevator.state = far_KL3L_RFFS_CD3L_RFFS_AB2LElevatorStates[2];
                            thisRobot.drivebase.initPathToPoint(far_KL3L_RFFS_CD3L_RFFS_AB2LAutoPoses[2]);
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
                        thisRobot.drivebase.swerveDrive.lockPose();
                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.5) {
                            thisRobot.coral.setMotorPower();
                            thisRobot.drivebase.initPathToPoint(far_KL3L_RFFS_CD3L_RFFS_AB2LAutoPoses[3]);
                            autoStep = 20;
                            thisRobot.elevator.state = far_KL3L_RFFS_CD3L_RFFS_AB2LElevatorStates[3];
                        }
                        break;
                    case 35:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        if (thisRobot.drivebase.followOTFPath()) {
                            autoStep = 40;
                            thisRobot.elevator.state = far_KL3L_RFFS_CD3L_RFFS_AB2LElevatorStates[4];
                            thisRobot.drivebase.initPathToPoint(far_KL3L_RFFS_CD3L_RFFS_AB2LAutoPoses[4]);
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
            case mid_GH4L_RCFS_EF4L_RCFS_CD4L:
                switch (autoStep) {
                    case 0:
                        firstAutoBeingRun = false;
                        if (Robot.isSimulation()) {
                            if (thisRobot.onRedAlliance) {
                                thisRobot.drivebase.swerveDrive.resetOdometry(
                                        thisRobot.drivebase.flipPoseToRed(mid_GH4L_RCFS_EF4L_RCFS_CD4LStartPose));
                            } else {
                                thisRobot.drivebase.swerveDrive
                                        .resetOdometry(mid_GH4L_RCFS_EF4L_RCFS_CD4LStartPose);
                            }
                        }
                        thisRobot.drivebase.initPathToPoint(mid_GH4L_RCFS_EF4L_RCFS_CD4LAutoPoses[0]);
                        thisRobot.coral.state = CoralIntakeStates.stationary;
                        thisRobot.elevator.state = mid_GH4L_RCFS_EF4L_RCFS_CD4LElevatorStates[0];
                        autoStep = 5;
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
                            thisRobot.drivebase.initPathToPoint(mid_GH4L_RCFS_EF4L_RCFS_CD4LAutoPoses[1]);
                            autoStep = 20;
                            thisRobot.elevator.state = mid_GH4L_RCFS_EF4L_RCFS_CD4LElevatorStates[1];
                            thisRobot.coral.state = CoralIntakeStates.outake;
                        }
                        break;
                    case 20:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        if (thisRobot.drivebase.followOTFPath()) {
                            autoStep = 25;
                            thisRobot.elevator.state = mid_GH4L_RCFS_EF4L_RCFS_CD4LElevatorStates[2];
                            thisRobot.drivebase.initPathToPoint(mid_GH4L_RCFS_EF4L_RCFS_CD4LAutoPoses[2]);
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
                        thisRobot.drivebase.swerveDrive.lockPose();
                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.5) {
                            thisRobot.coral.setMotorPower();
                            thisRobot.drivebase.initPathToPoint(mid_GH4L_RCFS_EF4L_RCFS_CD4LAutoPoses[3]);
                            autoStep = 20;
                            thisRobot.elevator.state = mid_GH4L_RCFS_EF4L_RCFS_CD4LElevatorStates[3];
                        }
                        break;
                    case 35:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        if (thisRobot.drivebase.followOTFPath()) {
                            autoStep = 40;
                            thisRobot.elevator.state = mid_GH4L_RCFS_EF4L_RCFS_CD4LElevatorStates[4];
                            thisRobot.drivebase.initPathToPoint(mid_GH4L_RCFS_EF4L_RCFS_CD4LAutoPoses[4]);
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
