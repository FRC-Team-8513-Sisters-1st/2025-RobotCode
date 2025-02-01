package frc.robot.logic;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Dashboard {

    Robot thisRobot;
    Field2d dashboardField2d = new Field2d();

    public Dashboard(Robot thisRobotIn) {
        
        thisRobot = thisRobotIn;
    }
    
    public void updateDashboard(){
        putPoseOnDashboard("goalFeederStation", thisRobot.stateMachine.goalFeederStation);
        putPoseOnDashboard("goalProcessor", thisRobot.stateMachine.goalProcessor);
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
        SmartDashboard.putBoolean("elevatorButtonPressed", thisRobot.stateMachine.elevatorButtonPressed);
        SmartDashboard.putString("operatorChosenSideOfReef", thisRobot.stateMachine.operatorChosenSideOfReef.name());
    }

    public void putPoseOnDashboard( String poseName, Pose2d pose2d) {
        dashboardField2d.setRobotPose(pose2d);
        SmartDashboard.putData(poseName, dashboardField2d);
    }
}
