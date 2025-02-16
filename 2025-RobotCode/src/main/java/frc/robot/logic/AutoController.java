package frc.robot.logic;

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

    public void autoDis(){
        autoRoutine = AutoRoutines.valueOf(autoSelector.getSelected());
    }

    public void autoPeriodic() {
        switch (autoRoutine) {
            case DoNothing:
                thisRobot.drivebase.swerveDrive.lockPose();
                break;
            case SamplePathThenAP:
                switch (autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("circle while turning");
                        autoStep = 10;
                        break;
                    case 10:
                        if (thisRobot.drivebase.followLoadedPath()) {
                            autoStep = 20;
                        }
                        break;
                    case 20:
                        thisRobot.drivebase.attackPoint(Settings.coralLeftCD, 3);
                        break;
                    default:
                        break;
                }
                break;
            case FillerJustPath:
                switch (autoStep) {
                    case 0:
                        thisRobot.drivebase.initPath("circle while turning");
                        autoStep = 10;
                        break;
                    case 10:
                        if (thisRobot.drivebase.followLoadedPath()) {
                            autoStep = 0;
                        }
                        break;
                    case 20:
                        thisRobot.drivebase.swerveDrive.lockPose();
                        break;
                    default:
                        break;
                }
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
            default:
                break;
        }
    }

}
