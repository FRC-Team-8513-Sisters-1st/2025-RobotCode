package frc.robot.Logic;

import edu.wpi.first.math.geometry.Pose2d;
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
    public int autoStep = 0;
    public boolean firstAutoBeingRun = true;
    boolean generatedPathFirstTime = true;

    public SendableChooser<String> autoSelector;

    double timeStepStarted = 0;

    // custom auto
    Pose2d customAutoStartPose = Settings.autoProcessorStartPose;
    Pose2d[] customAutoPoses = { Settings.coralLeftEF,
            Settings.leftCenterFeederStationAP, Settings.coralRightAB,
            Settings.leftCenterFeederStationAP, Settings.coralLeftCD,
            Settings.leftCenterFeederStationAP, Settings.coralRightEF,
            Settings.rightCenterFeederStationAP, Settings.coralLeftGH,
            Settings.rightCenterFeederStationAP, Settings.coralRightIJ,
            Settings.rightCenterFeederStationAP, Settings.coralLeftAB };
    ElevatorStates[] customElevatorStates = { ElevatorStates.L4,
            ElevatorStates.L1, ElevatorStates.L4,
            ElevatorStates.L1, ElevatorStates.L4,
            ElevatorStates.L1, ElevatorStates.L4,
            ElevatorStates.L1, ElevatorStates.L4,
            ElevatorStates.L1, ElevatorStates.L4,
            ElevatorStates.L1, ElevatorStates.L4 };

    int customAutoStep = 0;

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
        customAutoStep = 0;
    }

    public void autoDis() {
        updateAutoRoutine();
    }

    public void updateAutoRoutine() {
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
                        Settings.rightCenterFeederStationAP, Settings.coralLeftAB, Settings.rightCenterFeederStationAP,
                        Settings.coralRightAB };
                customElevatorStates = new ElevatorStates[] { ElevatorStates.L2,
                        ElevatorStates.L1, ElevatorStates.L2,
                        ElevatorStates.L1, ElevatorStates.L2, };
                autoRoutine = AutoRoutines.customAutoAnyLength;
                break;
            case far_IJ2L_LFS_KL4R_LFS_KL4L:
                customAutoStartPose = Settings.autoFarStartPose;
                customAutoPoses = new Pose2d[] { Settings.coralLeftIJ,
                        Settings.leftCenterFeederStationAP, Settings.coralRightKL, Settings.leftCenterFeederStationAP,
                        Settings.coralLeftKL };
                customElevatorStates = new ElevatorStates[] { ElevatorStates.L2,
                        ElevatorStates.L1, ElevatorStates.L4,
                        ElevatorStates.L1, ElevatorStates.L4, };
                autoRoutine = AutoRoutines.customAutoAnyLength;
                break;
            case mid_EF2R_RFS_CD4R_RFS_CD4L:
                customAutoStartPose = Settings.autoMidStartPose;
                customAutoPoses = new Pose2d[] { Settings.coralRightEF,
                        Settings.rightCenterFeederStationAP, Settings.coralRightCD, Settings.rightCenterFeederStationAP,
                        Settings.coralLeftCD };
                customElevatorStates = new ElevatorStates[] { ElevatorStates.L2,
                        ElevatorStates.L1, ElevatorStates.L4,
                        ElevatorStates.L1, ElevatorStates.L4, };
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
                        ElevatorStates.L1, ElevatorStates.L2};
                autoRoutine = AutoRoutines.customAutoAnyLength;
                break;
            case customAutoAnyLength:
                switch (autoStep) {
                    case 0:
                        if (Robot.isSimulation()) {
                            if (thisRobot.onRedAlliance) {
                                thisRobot.drivebase.swerveDrive.resetOdometry(
                                        thisRobot.drivebase.flipPoseToRed(customAutoStartPose));
                            } else {
                                thisRobot.drivebase.swerveDrive.resetOdometry(customAutoStartPose);
                            }
                        }
                        thisRobot.drivebase.initAstarAndAP(
                                customAutoPoses[customAutoStep].transformBy(Settings.astarReefPoseOffset),
                                customAutoPoses[0]);
                        thisRobot.coral.state = CoralIntakeStates.stationary;
                        thisRobot.elevator.state = customElevatorStates[customAutoStep];
                        customAutoStep++;
                        autoStep = 5;
                        // intentially no break
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
                        if (Timer.getFPGATimestamp() - timeStepStarted > 0.3) {
                            thisRobot.coral.setMotorPower();
                            thisRobot.drivebase.initAstarAndAP(
                                    customAutoPoses[customAutoStep].transformBy(Settings.astarFeederStPoseOffset),
                                    customAutoPoses[customAutoStep]);
                            customAutoStep++;
                            autoStep = 15;
                            thisRobot.elevator.state = customElevatorStates[customAutoStep];
                            thisRobot.coral.state = CoralIntakeStates.outake;

                        }
                        break;
                    case 15:
                        thisRobot.elevator.setMotorPower();
                        thisRobot.coral.setMotorPower();
                        if (thisRobot.drivebase.fromOTFSwitchToAP()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 16;
                        }
                        break;
                    case 16:
                    if (Timer.getFPGATimestamp() - timeStepStarted > 0.5) {
                        autoStep = 20;
                    }
                    break;
                    case 20:
                        if (autoScoreCoral(customAutoPoses[customAutoStep], customElevatorStates[customAutoStep])) {
                            if (customAutoStep >= customAutoPoses.length - 1) {
                                autoStep = 45;
                            } else {
                                autoStep = 10;
                                customAutoStep++;
                            }
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
            default:
                break;
        }
    }
    public boolean autoScoreCoral(Pose2d goalPose, ElevatorStates elevatorState) {
        boolean isComplete = false;

        thisRobot.elevator.setMotorPower();
        thisRobot.coral.setMotorPower();
        if (generatedPathFirstTime) {
            thisRobot.elevator.state = elevatorState;
            thisRobot.drivebase.initAstarAndAP(
                    goalPose.transformBy(Settings.astarReefPoseOffset),
                    goalPose);
            generatedPathFirstTime = false;
        }
        if (thisRobot.drivebase.fromOTFSwitchToAP()) {
            isComplete = true;
            generatedPathFirstTime = true;
            timeStepStarted = Timer.getFPGATimestamp();
            thisRobot.coral.state = CoralIntakeStates.outake;
        }
        return isComplete;
    }
}
