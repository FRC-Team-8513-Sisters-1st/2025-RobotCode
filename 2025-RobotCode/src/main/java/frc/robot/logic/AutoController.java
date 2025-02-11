package frc.robot.logic;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.logic.Enums.AutoRoutines;


public class AutoController {

    Robot thisRobot;
    AutoRoutines autoRoutine = AutoRoutines.DoNothing;
    int autoStep = 0;

    public SendableChooser<String> autoSelector;

    public AutoController(Robot thisRobotIn){
        thisRobot = thisRobotIn;

        // create auto selector with each enum option
        autoSelector = new SendableChooser<>();
        autoSelector.setDefaultOption(AutoRoutines.values()[0].toString(), AutoRoutines.values()[0].toString());
        for (int i = 1; i < AutoRoutines.values().length; i++) {
            if(AutoRoutines.values()[i].toString().charAt(0) != '_'){
                autoSelector.addOption(AutoRoutines.values()[i].toString(), AutoRoutines.values()[i].toString());
            }
            
        }
        SmartDashboard.putData("Auton Selector", autoSelector);

    }

    public void autoInit() {
        autoRoutine = AutoRoutines.valueOf(autoSelector.getSelected());
        SmartDashboard.putString("AutoRoutine", autoRoutine.toString());
        autoStep = 0;
    }

    public void autoPeriodic() {
        SmartDashboard.putNumber("autoStep", autoStep);
        switch(autoRoutine) {
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
                            autoStep = 0;
                        }
                        break;
                    case 20:
                        thisRobot.drivebase.attackPoint(Settings.coralLeftCD);
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
            default:
              break;
        }
    }   
    
}

