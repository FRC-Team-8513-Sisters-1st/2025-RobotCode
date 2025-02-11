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

    public void updateDashboard() {
        putPoseOnDashboard("goalFeederStation", thisRobot.stateMachine.goalFeederStation);
        putPoseOnDashboard("goalProcessor", thisRobot.stateMachine.goalProcessor);
        putPoseOnDashboard("coralScoreGoalPose", thisRobot.teleopController.coralScoreGoalPose);
        putPoseOnDashboard("rightFarFeederStation", thisRobot.stateMachine.goalFeederStation);
        putPoseOnDashboard("processor", thisRobot.stateMachine.goalFeederStation);
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
        SmartDashboard.putBoolean("isInReefZone", thisRobot.stateMachine.isRobotInReefZone());
        SmartDashboard.putString("feederCloseOrFar", thisRobot.stateMachine.feederCloseOrFar.name());
        SmartDashboard.putBoolean("climberButtonPressed", thisRobot.stateMachine.climberButtonPressed);
        SmartDashboard.putString("operatorChosenSideOfReef", thisRobot.stateMachine.operatorChosenSideOfReef.name());
        SmartDashboard.putNumber("elevator motor 1 power", thisRobot.elevator.elevatorMotor1.getAppliedOutput());
        SmartDashboard.putNumber("elevator motor 2 power", thisRobot.elevator.elevatorMotor2.getAppliedOutput());
        SmartDashboard.putNumber("elevator motor 1 pos", thisRobot.elevator.elevatorMotor1.getEncoder().getPosition());
        SmartDashboard.putNumber("elevator motor 2 pos", thisRobot.elevator.elevatorMotor2.getEncoder().getPosition());
        SmartDashboard.putNumber("elevatorGoalState", thisRobot.elevator.m_controller.getSetpoint().position);
        SmartDashboard.putBoolean("isRobotinReefZone", thisRobot.stateMachine.isRobotInReefZone());
        SmartDashboard.putNumber("storedElevatorState", thisRobot.elevator.storedElevatorState);
        SmartDashboard.putNumber("coral motor current", thisRobot.coral.coralMotor1.getOutputCurrent());

    }

    public void putPoseOnDashboard(String poseName, Pose2d pose2d) {
        dashboardField2d.setRobotPose(pose2d);
        SmartDashboard.putData(poseName, dashboardField2d);
    }
}
