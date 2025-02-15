package frc.robot.logic;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Dashboard {

    Robot thisRobot;
    Field2d coralGoaField2d = new Field2d();
    Field2d goalFeederStationField2d = new Field2d();
    Field2d processorField2d = new Field2d();
    public Field2d attackPoitnField2d = new Field2d();

    public Dashboard(Robot thisRobotIn) {

        thisRobot = thisRobotIn;

        // fields only need to be put once, they auto update when you update the pose
        SmartDashboard.putData("goalFeederStation", goalFeederStationField2d);
        SmartDashboard.putData("coralScoreGoalPose", coralGoaField2d);
        SmartDashboard.putData("processor", processorField2d);
        SmartDashboard.putData("attackPointPose", attackPoitnField2d);
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("Time", Timer.getFPGATimestamp());
        // elevator motor stats
        SmartDashboard.putNumber("elevator motor 1 power", thisRobot.elevator.elevatorMotor1.getAppliedOutput());
        SmartDashboard.putNumber("elevator motor 2 power", thisRobot.elevator.elevatorMotor2.getAppliedOutput());
        SmartDashboard.putNumber("elevator motor 1 pos", thisRobot.elevator.elevatorMotor1.getEncoder().getPosition());
        SmartDashboard.putNumber("elevator motor 2 pos", thisRobot.elevator.elevatorMotor2.getEncoder().getPosition());
        SmartDashboard.putNumber("elevatorGoalState", thisRobot.elevator.m_controller.getGoal().position);
        SmartDashboard.putNumber("elevatorPosition", thisRobot.elevator.elevatorMotor1.getEncoder().getPosition());
        SmartDashboard.putString("elevatorState", thisRobot.elevator.state.name());

        // coral mech stats
        SmartDashboard.putNumber("coral motor current", thisRobot.coral.coralMotor1.getOutputCurrent());
        SmartDashboard.putNumber("coralSensor", thisRobot.coral.coralMotor1.getAnalog().getVoltage());
        SmartDashboard.putString("coralState", thisRobot.coral.state.name());

    }
}
