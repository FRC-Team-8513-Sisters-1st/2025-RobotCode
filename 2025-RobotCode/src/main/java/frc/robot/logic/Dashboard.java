package frc.robot.logic;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Dashboard {

    Robot thisRobot;

    public Dashboard(Robot thisRobotIn) {
        
        thisRobot = thisRobotIn;
    }
    
    public void updateDashboard(){
        SmartDashboard.putNumber("Time", Timer.getFPGATimestamp());
        SmartDashboard.putBoolean("updateIsRobotInReefZone", thisRobot.stateMachine.isRobotInReefZone());
        SmartDashboard.putString("robotState", thisRobot.stateMachine.robotState.name());
        SmartDashboard.putString("climberStates", thisRobot.stateMachine.climberStates.name());
        SmartDashboard.putString("algaeGroundStates", thisRobot.stateMachine.algaeGroundStates.name());
        SmartDashboard.putString("algaeIntakeStates", thisRobot.stateMachine.algaeIntakeStates.name());
        SmartDashboard.putString("coralIntakeStates", thisRobot.stateMachine.coralIntakeStates.name());
        SmartDashboard.putString("elevatorStates", thisRobot.stateMachine.elevatorStates.name());
        SmartDashboard.putString("drivebaseStates", thisRobot.stateMachine.drivebaseStates.name());
        SmartDashboard.putString("scoreCoralGoalLevel", thisRobot.stateMachine.scoreCoralGoalLevel.name());
        SmartDashboard.putBoolean("isInReefZone", thisRobot.stateMachine.isInReefZone);
        SmartDashboard.putString("feederCloseOrFar", thisRobot.stateMachine.feederCloseOrFar.name());
        // for pose2d make a field2d and directly put the field on the dashboard.
        SmartDashboard.putBoolean("elevatorButtonPressed", thisRobot.stateMachine.elevatorButtonPressed);
        SmartDashboard.putString("operatorChosenSideOfReef", thisRobot.stateMachine.operatorChosenSideOfReef.name());
    }
}
