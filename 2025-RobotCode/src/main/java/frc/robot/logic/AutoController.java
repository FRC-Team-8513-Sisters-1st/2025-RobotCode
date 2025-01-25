package frc.robot.logic;

import frc.robot.logic.Enums.RobotStates;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.logic.Enums.AutoRoutines;


public class AutoController {

    Robot thisRobot;
    AutoRoutines autoRoutines = AutoRoutines.DoNothing;
    String path1 = "";
    StateMachine stateMachine;
    RobotStates robotStates = RobotStates.driving;

    public AutoController(Robot thisRobotIn){
        thisRobot = thisRobotIn;
    }

    public void autoInit() {
        SmartDashboard.putString("AutoRoutine", autoRoutines.toString());
    }

    public void autoPeriodic() {
        switch(autoRoutines) {
            case DoNothing:
                    thisRobot.drivebase.swerveDrive.lockPose();
                break;
            case Filler:
                path1 = "AmpStartToNote3ToAmpShot";
              break;
            default:
              break;
        }
    }   
    
}

