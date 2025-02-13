package frc.robot.logic;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Dashboard {

    Robot thisRobot;
    Field2d dashboardField2d = new Field2d();
    Field2d coralGoaField2d = new Field2d();
    Field2d goalFeederStationField2d = new Field2d();
    Field2d goalProcessorField2d = new Field2d();
    Field2d rightFarFeederStationField2d = new Field2d();
    Field2d rightCloseFeederStationField2d = new Field2d();
    Field2d leftFarFeederStationField2d = new Field2d();
    Field2d leftCloseFeederStationField2d = new Field2d();
    Field2d processorField2d = new Field2d();

    public Dashboard(Robot thisRobotIn) {

        thisRobot = thisRobotIn;

        //fields only need to be put once, they auto update when you update the pose
        SmartDashboard.putData("leftCloseFeederStation", leftCloseFeederStationField2d);
        SmartDashboard.putData("goalFeederStation", goalFeederStationField2d);
        SmartDashboard.putData("goalProcessor", goalProcessorField2d);
        SmartDashboard.putData("coralScoreGoalPose", coralGoaField2d);
        SmartDashboard.putData("rightFarFeederStation", rightFarFeederStationField2d);
        SmartDashboard.putData("rightCloseFeedeerStation", rightCloseFeederStationField2d);
        SmartDashboard.putData("leftFarFeederStation", leftFarFeederStationField2d);
        SmartDashboard.putData("processor", processorField2d);
    }

    public void updateDashboard() {

        // poses
        goalFeederStationField2d.setRobotPose(thisRobot.stateMachine.goalFeederStation);
        goalProcessorField2d.setRobotPose(thisRobot.stateMachine.goalProcessor);
        coralGoaField2d.setRobotPose(thisRobot.teleopController.coralScoreGoalPose);
        rightFarFeederStationField2d.setRobotPose(thisRobot.stateMachine.goalFeederStation);
        rightCloseFeederStationField2d.setRobotPose(thisRobot.stateMachine.goalFeederStation);
        leftFarFeederStationField2d.setRobotPose(thisRobot.stateMachine.goalFeederStation);
        leftCloseFeederStationField2d.setRobotPose(thisRobot.stateMachine.goalFeederStation);
        processorField2d.setRobotPose(thisRobot.stateMachine.goalFeederStation);

        // Robot statistics
        SmartDashboard.putNumber("Time", Timer.getFPGATimestamp());
        SmartDashboard.putString("robotState", thisRobot.stateMachine.robotState.name());

        // subsystem states
        
        SmartDashboard.putString("climberStates", thisRobot.stateMachine.climberStates.name());
        SmartDashboard.putString("algaeGroundStates", thisRobot.stateMachine.algaeGroundStates.name());
        SmartDashboard.putString("algaeIntakeStates", thisRobot.stateMachine.algaeIntakeStates.name());
        SmartDashboard.putString("coralIntakeStates", thisRobot.stateMachine.coralIntakeStates.name());
        SmartDashboard.putString("elevatorStates", thisRobot.stateMachine.elevatorStates.name());
        SmartDashboard.putString("drivebaseStates", thisRobot.stateMachine.drivebaseStates.name());

        // scoring variables
        SmartDashboard.putString("scoreCoralGoalLevel", thisRobot.stateMachine.scoreCoralGoalLevel.name());
        SmartDashboard.putBoolean("isInReefZone", thisRobot.stateMachine.isRobotInReefZone());
        SmartDashboard.putString("feederCloseOrFar", thisRobot.stateMachine.feederCloseOrFar.name());
        SmartDashboard.putBoolean("climberButtonPressed", thisRobot.stateMachine.climberButtonPressed);
        SmartDashboard.putString("operatorChosenSideOfReef", thisRobot.stateMachine.operatorChosenSideOfReef.name());

        // elevator motor stats
        SmartDashboard.putNumber("elevator motor 1 power", thisRobot.elevator.elevatorMotor1.getAppliedOutput());
        SmartDashboard.putNumber("elevator motor 2 power", thisRobot.elevator.elevatorMotor2.getAppliedOutput());
        SmartDashboard.putNumber("elevator motor 1 pos", thisRobot.elevator.elevatorMotor1.getEncoder().getPosition());
        SmartDashboard.putNumber("elevator motor 2 pos", thisRobot.elevator.elevatorMotor2.getEncoder().getPosition());
        SmartDashboard.putNumber("elevatorGoalState", thisRobot.elevator.m_controller.getGoal().position);
        SmartDashboard.putNumber("elevatorPosition", thisRobot.elevator.elevatorMotor1.getEncoder().getPosition());
        SmartDashboard.putNumber("storedElevatorState", thisRobot.elevator.storedElevatorState);

        // coral mech stats
        SmartDashboard.putNumber("coral motor current", thisRobot.coral.coralMotor1.getOutputCurrent());
        SmartDashboard.putNumber("coralSensor", thisRobot.coral.coralMotor1.getAnalog().getVoltage());

    }
}
