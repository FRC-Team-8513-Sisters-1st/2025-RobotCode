package frc.robot.Logic;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Logic.Enums.AutoRoutines;
import frc.robot.Logic.Enums.CoralIntakeStates;
import frc.robot.Logic.Enums.ElevatorStates;

public class AutoController {

    Robot thisRobot;
    public AutoRoutines autoRoutine = AutoRoutines.DoNothing;
    public SendableChooser<String> autoSelector;

    public boolean firstAutoBeingRun = true;
    boolean generatedPathFirstTime = true;
    double timeStepStarted = 0;

    public int autoStep = 0;
    int customAutoStep = 0;

    // custom auto
    Pose2d customAutoStartPose = Settings.autoProcessorStartPose;
    Pose2d[] customAutoPoses = { Settings.coralLeftEF,
            Settings.leftCenterFeederStationAP, Settings.coralLeftAB,
            Settings.leftCenterFeederStationAP, Settings.coralLeftCD,
            Settings.leftCenterFeederStationAP, Settings.coralLeftEF,
            Settings.leftCenterFeederStationAP, Settings.coralLeftGH,
            Settings.leftCenterFeederStationAP, Settings.coralLeftIJ,
            Settings.leftCenterFeederStationAP, Settings.coralLeftKL,
            Settings.rightCenterFeederStationAP, Settings.coralRightAB,
            Settings.rightCenterFeederStationAP, Settings.coralRightCD,
            Settings.rightCenterFeederStationAP, Settings.coralRightEF,
            Settings.rightCenterFeederStationAP, Settings.coralRightGH,
            Settings.rightCenterFeederStationAP, Settings.coralRightIJ,
            Settings.rightCenterFeederStationAP, Settings.coralRightKL };
    ElevatorStates[] customElevatorStates = { ElevatorStates.L4,
            ElevatorStates.L1, ElevatorStates.L4,
            ElevatorStates.L1, ElevatorStates.L4,
            ElevatorStates.L1, ElevatorStates.L4,
            ElevatorStates.L1, ElevatorStates.L4,
            ElevatorStates.L1, ElevatorStates.L4,
            ElevatorStates.L1, ElevatorStates.L4,
            ElevatorStates.L1, ElevatorStates.L4,
            ElevatorStates.L1, ElevatorStates.L4,
            ElevatorStates.L1, ElevatorStates.L4,
            ElevatorStates.L1, ElevatorStates.L4,
            ElevatorStates.L1, ElevatorStates.L4,
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
        updateAutoRoutineFromDashboard();
        autoStep = 0;
        customAutoStep = 0;
        thisRobot.elevator.autoElevatorOn = true;
        thisRobot.vision.useProcessorCam = false;
        thisRobot.vision.updateHeadingWithVision = false;
        thisRobot.vision.visionMaxATDist = Settings.maxATDist;
    }

    public void autoDis() {
        updateAutoRoutineFromDashboard();
    }

    public void updateAutoRoutineFromDashboard() {
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

            case processor_EF2L_RFS_AB2L_RFS_AB2R:
                customAutoStartPose = Settings.autoProcessorStartPose;
                customAutoPoses = new Pose2d[] { Settings.coralLeftEF,
                        Settings.rightCenterFeederStationAP, Settings.coralLeftAB,
                        Settings.rightCenterFeederStationAP, Settings.coralRightAB,
                        Settings.rightCenterFeederStationAP, Settings.coralLeftCD };
                customElevatorStates = new ElevatorStates[] { ElevatorStates.L2,
                        ElevatorStates.L1, ElevatorStates.L2,
                        ElevatorStates.L1, ElevatorStates.L2,
                        ElevatorStates.L1, ElevatorStates.L2 };
                autoRoutine = AutoRoutines.customAutoAnyLength;
                break;

            case far_IJ2L_LFS_KL4R_LFS_KL4L:
                customAutoStartPose = Settings.autoFarStartPose;
                customAutoPoses = new Pose2d[] { Settings.coralLeftIJ,
                        Settings.leftCenterFeederStationAP, Settings.coralRightKL,
                        Settings.leftCenterFeederStationAP, Settings.coralLeftKL,
                        Settings.leftCenterFeederStationAP, Settings.coralLeftAB };
                customElevatorStates = new ElevatorStates[] { ElevatorStates.L2,
                        ElevatorStates.L1, ElevatorStates.L4,
                        ElevatorStates.L1, ElevatorStates.L4,
                        ElevatorStates.L1, ElevatorStates.L4 };
                autoRoutine = AutoRoutines.customAutoAnyLength;
                break;

            case mid_EF2R_RFS_CD4R_RFS_CD4L:
                customAutoStartPose = Settings.autoMidStartPose;
                customAutoPoses = new Pose2d[] { Settings.coralRightEF,
                        Settings.rightCenterFeederStationAP, Settings.coralRightCD,
                        Settings.rightCenterFeederStationAP, Settings.coralLeftCD,
                        Settings.rightCenterFeederStationAP, Settings.coralRightAB };
                customElevatorStates = new ElevatorStates[] { ElevatorStates.L2,
                        ElevatorStates.L1, ElevatorStates.L4,
                        ElevatorStates.L1, ElevatorStates.L4,
                        ElevatorStates.L1, ElevatorStates.L4 };
                autoRoutine = AutoRoutines.customAutoAnyLength;
                break;

            case betWithBusler:
                customAutoStartPose = Settings.autoProcessorStartPose;
                customAutoPoses = new Pose2d[] { Settings.coralLeftEF,
                        Settings.rightCenterFeederStationAP, Settings.coralLeftAB,
                        Settings.rightCenterFeederStationAP, Settings.coralRightAB,
                        Settings.rightCenterFeederStationAP, Settings.coralLeftCD };
                customElevatorStates = new ElevatorStates[] { ElevatorStates.L2,
                        ElevatorStates.L1, ElevatorStates.L2,
                        ElevatorStates.L1, ElevatorStates.L2,
                        ElevatorStates.L1, ElevatorStates.L2 };
                autoRoutine = AutoRoutines.customAutoAnyLength;
                break;

            case customAutoAnyLength:
                switch (autoStep) {
                    case 0: // checks to flip to red side in simulation
                        if (Robot.isSimulation()) {
                            if (thisRobot.onRedAlliance) {
                                thisRobot.drivebase.swerveDrive.resetOdometry(
                                        thisRobot.drivebase.flipPoseToRed(customAutoStartPose));
                            } else {
                                thisRobot.drivebase.swerveDrive.resetOdometry(customAutoStartPose);
                            }
                        }
                        autoStep = 5;
                    case 5: // scores at initial scoring position
                        thisRobot.coral.state = CoralIntakeStates.stationary;
                        if (autoScoreCoral(customAutoPoses[customAutoStep], customElevatorStates[customAutoStep])) {
                            autoStep = 10;
                            customAutoStep++;
                        }
                        break;
                    case 10: // waits to finish scoring, then generates path to coral station
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        thisRobot.drivebase.swerveDrive.lockPose();
                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.3) {
                            thisRobot.coral.setMotorPower();
                            thisRobot.drivebase.initAstarAndAP(
                                    customAutoPoses[customAutoStep].transformBy(Settings.astarFeederStPoseOffset),
                                    customAutoPoses[customAutoStep]);
                            autoStep = 15;
                            thisRobot.elevator.state = customElevatorStates[customAutoStep];
                            customAutoStep++;
                            thisRobot.coral.state = CoralIntakeStates.outake;

                        }
                        break;
                    case 15: // drives to coral station
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        thisRobot.coral.state = CoralIntakeStates.outake;
                        if (thisRobot.drivebase.fromOTFSwitchToAP()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 16;
                        }
                        break;
                    case 16: // waits at coral station
                        thisRobot.drivebase.fromOTFSwitchToAP();
                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.35) {
                            autoStep = 20;
                        }
                        break;
                    case 20: // generates path and scores at desired scoring location, then chooses whether
                             // to continue or end auto
                        if (autoScoreCoral(customAutoPoses[customAutoStep], customElevatorStates[customAutoStep])) {
                            if (customAutoStep >= customAutoPoses.length - 1) {
                                autoStep = 45;
                            } else {
                                autoStep = 10;
                                customAutoStep++;
                            }
                        }
                        break;
                    case 45: // ends auto
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        thisRobot.drivebase.swerveDrive.lockPose();
                        break;

                    default:
                        break;
                }
                break;
            case hpPractice:
                switch (autoStep) {
                    case 0: // checks to flip to red side in simulation
                        if (Robot.isSimulation()) {
                            if (thisRobot.onRedAlliance) {
                                thisRobot.drivebase.swerveDrive.resetOdometry(
                                        thisRobot.drivebase.flipPoseToRed(Settings.rightCenterFeederStationAP));
                            } else {
                                thisRobot.drivebase.swerveDrive.resetOdometry(Settings.rightCenterFeederStationAP);
                            }
                        }
                        autoStep = 5;
                    case 5: // scores at initial scoring position
                        thisRobot.coral.state = CoralIntakeStates.outake;
                        if (autoScoreCoral(Settings.rightCenterFeederStationAP
                                .transformBy(new Transform2d(2, Math.random() * 2 - 1, new Rotation2d())),
                                ElevatorStates.L1)) {
                            autoStep = 10;
                            customAutoStep++;
                        }
                        break;
                    case 10: // waits to finish scoring, then generates path to coral station
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        thisRobot.drivebase.swerveDrive.lockPose();
                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.5) {
                            thisRobot.coral.setMotorPower();
                            thisRobot.drivebase.initAstarAndAP(
                                    Settings.rightCenterFeederStationAP.transformBy(Settings.astarFeederStPoseOffset),
                                    Settings.rightCenterFeederStationAP);
                            autoStep = 15;
                            thisRobot.elevator.state = ElevatorStates.L1;
                            customAutoStep++;
                            thisRobot.coral.state = CoralIntakeStates.outake;

                        }
                        break;
                    case 15: // drives to coral station
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        if (thisRobot.drivebase.fromOTFSwitchToAP()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 5;
                        }
                        break;

                    default:
                        break;
                }
                break;
            default:
                break;
        }
    }

    public boolean autoScoreCoral(Pose2d goalPose, ElevatorStates elevatorState) {
        boolean isComplete = false;

        thisRobot.elevator.setMotorPower();
        thisRobot.coral.setMotorPower();
        if (generatedPathFirstTime) { // generates path a single time
            thisRobot.elevator.state = elevatorState;
            thisRobot.drivebase.initAstarAndAP(
                    goalPose.transformBy(Settings.astarReefPoseOffset),
                    goalPose);
            generatedPathFirstTime = false;
        }
        if (thisRobot.drivebase.fromOTFSwitchToAP() && thisRobot.elevator.elevatorAtSetpoint()) { // drives to desired
                                                                                                  // scoring position
            isComplete = true;
            generatedPathFirstTime = true;
            timeStepStarted = Timer.getFPGATimestamp(); 
            //if we reach AP and are still outaking it means we dont have coral and we need to immedialy go back
            if (thisRobot.coral.state == CoralIntakeStates.outake && Robot.isReal()) {
                timeStepStarted = 0;
            }
            thisRobot.coral.state = CoralIntakeStates.outake;
        }
        return isComplete;
    }
}
