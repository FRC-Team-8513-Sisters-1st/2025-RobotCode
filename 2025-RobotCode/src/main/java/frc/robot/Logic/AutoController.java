package frc.robot.Logic;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Logic.Enums.AlgaeIntakeStates;
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
    double autoStartWaitTime = 0;

    boolean autoRan = false;

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
            if (AutoRoutines.values()[i].toString().charAt(0) != '~') {
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
        timeStepStarted = Timer.getFPGATimestamp();
        thisRobot.coral.coralController.reset();
        thisRobot.coral.coralMotor1.getEncoder().setPosition(0);
        Settings.elevatorSafeToGoThold = Settings.elevatorSafeToGoTholdAuto;
        autoRan = true;

    }

    public void autoDis() {
        updateAutoRoutineFromDashboard();
        SmartDashboard.putString("AutoMode", autoRoutine.name());
        if (autoRan == false) {
            autoPeriodic();
            autoStep = 0;
            customAutoStep = 0;
            // pre load the path
            thisRobot.drivebase.initAstarAndAP(
                    customAutoPoses[0].transformBy(Settings.astarFeederStPoseOffset),
                    customAutoPoses[0]);
        }

        if (Robot.isSimulation()) {
            if (thisRobot.onRedAlliance) {
                thisRobot.drivebase.swerveDrive.resetOdometry(
                        thisRobot.drivebase.flipPoseToRed(customAutoStartPose));
            } else {
                thisRobot.drivebase.swerveDrive.resetOdometry(customAutoStartPose);
            }
        }

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

            case processor_EF4L_RFS_AB4L_RFS_AB4R:
                customAutoStartPose = Settings.autoProcessorStartPose;
                customAutoPoses = new Pose2d[] { Settings.coralLeftEF,
                        Settings.rightCenterFeederStationAP, Settings.coralLeftAB,
                        Settings.rightCenterFeederStationAP, Settings.coralRightAB,
                        Settings.rightCenterFeederStationAP, Settings.coralLeftCD };
                customElevatorStates = new ElevatorStates[] { ElevatorStates.L4,
                        ElevatorStates.L1, ElevatorStates.L4,
                        ElevatorStates.L1, ElevatorStates.L4,
                        ElevatorStates.L1, ElevatorStates.L4 };
                autoRoutine = AutoRoutines.customAutoAnyLength;
                break;

            case processor_EF2L_RFS_CD4L_RFS_CD4R:
                customAutoStartPose = Settings.autoProcessorStartPose;
                customAutoPoses = new Pose2d[] { Settings.coralLeftEF,
                        Settings.rightCenterFeederStationAP, Settings.coralLeftCD,
                        Settings.rightCenterFeederStationAP, Settings.coralRightCD,
                        Settings.rightCenterFeederStationAP, Settings.coralLeftAB };
                customElevatorStates = new ElevatorStates[] { ElevatorStates.L2,
                        ElevatorStates.L1, ElevatorStates.L4,
                        ElevatorStates.L1, ElevatorStates.L4,
                        ElevatorStates.L1, ElevatorStates.L4 };
                autoRoutine = AutoRoutines.customAutoAnyLength;
                break;

            case _processor_EF4L_RFS_CD4L_RFS_CD4R:
                customAutoStartPose = Settings.autoProcessorStartPose;
                customAutoPoses = new Pose2d[] { Settings.coralRightEF,
                        Settings.rightCenterFeederStationAP, Settings.coralLeftCD,
                        Settings.rightCenterFeederStationAP, Settings.coralRightCD,
                        Settings.rightCenterFeederStationAP, Settings.coralRightAB };
                customElevatorStates = new ElevatorStates[] { ElevatorStates.L4,
                        ElevatorStates.L1, ElevatorStates.L4,
                        ElevatorStates.L1, ElevatorStates.L4,
                        ElevatorStates.L1, ElevatorStates.L4 };
                autoRoutine = AutoRoutines.customAutoAnyLength;
                break;
            case processor_CD4R_RFS_CD4L_RFS_AB4R:
                customAutoStartPose = Settings.autoProcessorStartPose;
                customAutoPoses = new Pose2d[] { Settings.coralRightCD,
                        Settings.rightCenterFeederStationAP, Settings.coralLeftCD,
                        Settings.rightCenterFeederStationAP, Settings.coralRightAB,
                        Settings.rightCenterFeederStationAP, Settings.coralLeftAB };
                customElevatorStates = new ElevatorStates[] { ElevatorStates.L4,
                        ElevatorStates.L1, ElevatorStates.L4,
                        ElevatorStates.L1, ElevatorStates.L4,
                        ElevatorStates.L1, ElevatorStates.L4 };
                autoRoutine = AutoRoutines.customAutoAnyLength;
                break;

            case far_IJ2L_LFS_KL2R_LFS_KL2L:
                customAutoStartPose = Settings.autoFarStartPose;
                customAutoPoses = new Pose2d[] { Settings.coralLeftIJ,
                        Settings.leftCenterFeederStationAP, Settings.coralRightKL,
                        Settings.leftCenterFeederStationAP, Settings.coralLeftKL,
                        Settings.leftCenterFeederStationAP, Settings.coralLeftAB };
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

            case far_KL4R_LFS_AB4L_LFS_AB4R:
                customAutoStartPose = Settings.autoFarStartPose;
                customAutoPoses = new Pose2d[] { Settings.coralRightKL,
                        Settings.leftCenterFeederStationAP, Settings.coralLeftAB,
                        Settings.leftCenterFeederStationAP, Settings.coralRightAB,
                        Settings.leftCenterFeederStationAP, Settings.coralRightKL };
                customElevatorStates = new ElevatorStates[] { ElevatorStates.L4,
                        ElevatorStates.L1, ElevatorStates.L4,
                        ElevatorStates.L1, ElevatorStates.L4,
                        ElevatorStates.L1, ElevatorStates.L4 };
                autoRoutine = AutoRoutines.customAutoAnyLength;
                break;

            case _far_IJ4R_LFS_KL4L_LFS_KL4R:
                customAutoStartPose = Settings.autoFarStartPose;
                customAutoPoses = new Pose2d[] { Settings.coralRightIJ,
                        Settings.leftCenterFeederStationAP, Settings.coralLeftKL,
                        Settings.leftCenterFeederStationAP, Settings.coralRightKL,
                        Settings.leftCenterFeederStationAP, Settings.coralLeftAB };
                customElevatorStates = new ElevatorStates[] { ElevatorStates.L4,
                        ElevatorStates.L1, ElevatorStates.L4,
                        ElevatorStates.L1, ElevatorStates.L4,
                        ElevatorStates.L1, ElevatorStates.L4 };
                autoRoutine = AutoRoutines.customAutoAnyLength;
                break;

            case mid_GH2R_RFS_CD4R_RFS_CD4L:
                customAutoStartPose = Settings.autoMidStartPose;
                customAutoPoses = new Pose2d[] { Settings.coralRightGH,
                        Settings.rightCenterFeederStationAP, Settings.coralRightCD,
                        Settings.rightCenterFeederStationAP, Settings.coralLeftCD,
                        Settings.rightCenterFeederStationAP, Settings.coralRightAB };
                customElevatorStates = new ElevatorStates[] { ElevatorStates.L2,
                        ElevatorStates.L1, ElevatorStates.L4,
                        ElevatorStates.L1, ElevatorStates.L4,
                        ElevatorStates.L1, ElevatorStates.L4 };
                autoRoutine = AutoRoutines.customAutoAnyLength;
                break;

            case mid_GH2R:
                customAutoStartPose = Settings.autoMidStartPose;
                customAutoPoses = new Pose2d[] { Settings.coralRightGH };
                customElevatorStates = new ElevatorStates[] { ElevatorStates.L2 };
                autoRoutine = AutoRoutines.customAutoAnyLength;
                break;

            case mid_GH4R:
                customAutoStartPose = Settings.autoMidStartPose;
                customAutoPoses = new Pose2d[] { Settings.coralRightGH };
                customElevatorStates = new ElevatorStates[] { ElevatorStates.L4 };
                autoRoutine = AutoRoutines.customAutoAnyLength;
                break;

            case mid_GH2R_LFS_KL4L_LFS_KL4R:
                customAutoStartPose = Settings.autoMidStartPose;
                customAutoPoses = new Pose2d[] { Settings.coralRightGH,
                        Settings.leftCenterFeederStationAP, Settings.coralLeftKL,
                        Settings.leftCenterFeederStationAP, Settings.coralRightKL,
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

            case mid_IJ2L_LFS_KL4L_LFS_KL4R:
                customAutoStartPose = Settings.autoMidStartPose;
                customAutoPoses = new Pose2d[] { Settings.coralLeftIJ,
                        Settings.leftCenterFeederStationAP, Settings.coralLeftKL,
                        Settings.leftCenterFeederStationAP, Settings.coralRightKL,
                        Settings.leftCenterFeederStationAP, Settings.coralLeftAB };
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

            case mid_GH4R_GH_P_EF_P:
                switch (autoStep) {
                    case 0: // checks to flip to red side in simulation
                        if (Robot.isSimulation()) {
                            if (thisRobot.onRedAlliance) {
                                thisRobot.drivebase.swerveDrive.resetOdometry(
                                        thisRobot.drivebase.flipPoseToRed(Settings.autoMidStartPose));
                            } else {
                                thisRobot.drivebase.swerveDrive.resetOdometry(Settings.autoMidStartPose);
                            }
                        }
                        if (Timer.getFPGATimestamp() - timeStepStarted > autoStartWaitTime) {
                            autoStep = 5;
                        }
                        break;
                    case 5: // scores at initial scoring position

                        thisRobot.coral.state = CoralIntakeStates.stationary;
                        if (autoScoreCoral(Settings.coralRightGH, ElevatorStates.L4)) {
                            autoStep = 10;
                            customAutoStep++;
                            timeStepStarted = Timer.getFPGATimestamp();
                        }
                        break;
                    case 10: // waits to finish scoring, then aps back 0.5 m and lowers elevator
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        thisRobot.drivebase.swerveDrive.lockPose();
                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.275) {
                            thisRobot.coral.forceOutake = false;
                            timeStepStarted = Timer.getFPGATimestamp();

                            thisRobot.drivebase.initAstarAndAP(
                                thisRobot.drivebase.swerveDrive.getPose(),
                                    thisRobot.drivebase.swerveDrive.getPose()
                                            .transformBy(new Transform2d(-0.6, 0, new Rotation2d())));
                            autoStep = 15;
                            thisRobot.drivebase.resetAPPIDControllers();

                            State elevatorGoalPIDState = new State(Settings.elevatorPosProcessor, 0);
                            thisRobot.elevator.m_controller.setGoal(elevatorGoalPIDState);  
                            thisRobot.coral.state = CoralIntakeStates.stationary;
                            thisRobot.algae.algaeState = AlgaeIntakeStates.intake;
                        }

                        break;
                    case 15: // backup
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        thisRobot.algae.setMotorPower();
                        // once backed up, drive back in at lower level

                        if (thisRobot.drivebase.fromOTFSwitchToAP()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 16;
                            thisRobot.drivebase.resetAPPIDControllers();
                            thisRobot.drivebase.initAstarAndAP(
                                thisRobot.drivebase.swerveDrive.getPose().transformBy(new Transform2d(0.01,0, new Rotation2d())),
                                Settings.coralRightGH);
                            thisRobot.drivebase.resetAPPIDControllers();
                        }
                        break;
                    case 16: // go back to grab algae
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        thisRobot.algae.setMotorPower();
                        // once backed up, drive back in at lower level

                        if (thisRobot.drivebase.fromOTFSwitchToAP()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 18;
                        }
                        break;
                    case 18: // wait at point to grab algae
                        thisRobot.drivebase.fromOTFSwitchToAP();
                        thisRobot.coral.setMotorPower();
                        thisRobot.elevator.setMotorPower();
                        thisRobot.algae.setMotorPower();

                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.5) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 20;
                            thisRobot.drivebase.initAstarAndAP(
                                    Settings.processorAP.transformBy(Settings.astarProcesserPoseOffset),
                                    Settings.processorAP);
                        }
                        break;

                    case 20: // drive to processor
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        thisRobot.algae.setMotorPower();
                        // once backed up, drive back in at lower level
                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.25) {
                            State elevatorGoalPIDState = new State(Settings.elevatorPosProcessor, 0);
                            thisRobot.elevator.m_controller.setGoal(elevatorGoalPIDState);  
                        }
                        if (thisRobot.drivebase.fromOTFSwitchToAP()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            thisRobot.algae.algaeState = AlgaeIntakeStates.outake;
                            autoStep = 25;
                        }
                        break;
                    
                    case 25: // wait at processor scoring
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        thisRobot.algae.setMotorPower();

                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.5) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 30;
                            thisRobot.drivebase.initAstarAndAP(
                                Settings.coralRightEF.transformBy(Settings.astarReefPoseOffset),
                                Settings.coralRightEF);
                            thisRobot.elevator.state = ElevatorStates.L2a;
                            thisRobot.algae.algaeState = AlgaeIntakeStates.intake;
                        }
                        break;
                    case 30: // drive back to reef
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        thisRobot.algae.setMotorPower();

                        if (thisRobot.drivebase.fromOTFSwitchToAP()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 35;
                        }
                        break;
                    case 35: // wait at point to grab algae
                        thisRobot.drivebase.fromOTFSwitchToAP();
                        thisRobot.coral.setMotorPower();
                        thisRobot.elevator.setMotorPower();
                        thisRobot.algae.setMotorPower();

                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.4) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 40;
                            thisRobot.drivebase.initAstarAndAP(
                                    Settings.processorAP.transformBy(Settings.astarProcesserPoseOffset),
                                    Settings.processorAP);
                        }
                        break;

                    case 40: // drive to processor
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        thisRobot.algae.setMotorPower();
                        // once backed up, drive back in at lower level
                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.25) {
                            State elevatorGoalPIDState = new State(Settings.elevatorPosProcessor, 0);
                            thisRobot.elevator.m_controller.setGoal(elevatorGoalPIDState);  
                        }
                        if (thisRobot.drivebase.fromOTFSwitchToAP()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            thisRobot.algae.algaeState = AlgaeIntakeStates.outake;
                            autoStep = 45;
                        }
                        break;
                    
                    case 45: // wait at processor scoring
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        thisRobot.algae.setMotorPower();

                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.5) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 99;
                        }
                        break;
                    case 99:
                        thisRobot.drivebase.swerveDrive.lockPose();
                        break;
                }
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
                        if (Timer.getFPGATimestamp() - timeStepStarted > autoStartWaitTime) {
                            autoStep = 5;
                        }
                        break;
                    case 5: // scores at initial scoring position

                        thisRobot.coral.state = CoralIntakeStates.stationary;
                        if (autoScoreCoral(customAutoPoses[customAutoStep], customElevatorStates[customAutoStep])) {
                            autoStep = 10;
                            customAutoStep++;
                            timeStepStarted = Timer.getFPGATimestamp();
                        }
                        break;
                    case 10: // waits to finish scoring, then generates path to coral station
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        thisRobot.drivebase.swerveDrive.lockPose();
                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.275) {
                            thisRobot.coral.forceOutake = false;
                            timeStepStarted = Timer.getFPGATimestamp();

                            if (customAutoStep >= customAutoPoses.length) {
                                thisRobot.coral.state = CoralIntakeStates.outake;
                                autoStep = 45;
                            } else {
                                thisRobot.coral.setMotorPower();
                                thisRobot.drivebase.initAstarAndAP(
                                        customAutoPoses[customAutoStep].transformBy(Settings.astarFeederStPoseOffset),
                                        customAutoPoses[customAutoStep]);
                                autoStep = 15;
                                thisRobot.elevator.state = customElevatorStates[customAutoStep];
                                customAutoStep++;
                                thisRobot.coral.state = CoralIntakeStates.outake;
                            }

                        }
                        break;
                    case 15: // drives to coral station
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        // for the 1 second after scoring keep forcing outake
                        if (Timer.getFPGATimestamp() - timeStepStarted < 0.5) {
                            thisRobot.coral.state = CoralIntakeStates.outake;
                            thisRobot.coral.sensorFirstTime = true;
                            thisRobot.coral.sensorBrokeThold = false;
                        }
                        if (thisRobot.drivebase.fromOTFSwitchToAP()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 16;
                        }
                        break;
                    case 16: // waits at coral station
                        thisRobot.drivebase.fromOTFSwitchToAP();
                        thisRobot.coral.setMotorPower();
                        thisRobot.elevator.setMotorPower();
                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.4) {
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
                    case 0: // checks to flip to red side in simulati on
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
                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.35) {
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
        boolean atAP = thisRobot.drivebase.fromOTFSwitchToAP();
        if (atAP && (thisRobot.elevator.newElevatorAtSetpoint() || thisRobot.coral.state == CoralIntakeStates.outake)) { // drives
                                                                                                                         // to
                                                                                                                         // desired
            // scoring position
            isComplete = true;
            generatedPathFirstTime = true;
            timeStepStarted = Timer.getFPGATimestamp();
            if (thisRobot.coral.state == CoralIntakeStates.stationary) {
                thisRobot.coral.state = CoralIntakeStates.outake;
                thisRobot.coral.forceOutake = true;
            }

        }
        return isComplete;
    }
}
