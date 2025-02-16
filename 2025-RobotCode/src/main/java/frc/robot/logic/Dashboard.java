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
        SmartDashboard.putData("attackPointPose", attackPoitnField2d);
    }

    public void updateDashboard() {
        //overall
        SmartDashboard.putNumber("Time", Timer.getFPGATimestamp());
        SmartDashboard.putBoolean("isOnRed", thisRobot.onRedAlliance);

        //autos
        SmartDashboard.putNumber("AutoStep", thisRobot.autoController.autoStep);
        SmartDashboard.putString("AutoMode", thisRobot.autoController.autoRoutine.name());
        SmartDashboard.putBoolean("FirstAutoRun", thisRobot.autoController.firstAutoBeingRun);
        
        //teleop 
        SmartDashboard.putString("Operator Chosen Side Of Reef", thisRobot.teleopController.operatorChosenSideOfReef.name());
        SmartDashboard.putString("Operator Chosen Side Of Feeder", thisRobot.teleopController.feederCloseOrFar.name());
        
        // elevator motor stats
        SmartDashboard.putNumber("elevator motor 1 power", thisRobot.elevator.elevatorMotor1.getAppliedOutput());
        SmartDashboard.putNumber("elevator motor 2 power", thisRobot.elevator.elevatorMotor2.getAppliedOutput());
        SmartDashboard.putNumber("elevator motor 1 pos", thisRobot.elevator.elevatorMotor1.getEncoder().getPosition());
        SmartDashboard.putNumber("elevator motor 2 pos", thisRobot.elevator.elevatorMotor2.getEncoder().getPosition());
        SmartDashboard.putNumber("elevatorGoalState", thisRobot.elevator.m_controller.getGoal().position);
        SmartDashboard.putNumber("elevatorPosition", thisRobot.elevator.elevatorMotor1.getEncoder().getPosition());
        SmartDashboard.putString("elevatorState", thisRobot.elevator.state.name());
        SmartDashboard.putBoolean("Auto Elevator On", thisRobot.elevator.autoElevatorOn);

        // coral mech stats
        SmartDashboard.putNumber("coral motor pos", thisRobot.coral.coralMotor1.getEncoder().getPosition());
        SmartDashboard.putNumber("coralSensor", thisRobot.coral.coralMotor1.getAnalog().getVoltage());
        SmartDashboard.putString("coralState", thisRobot.coral.state.name());

        //algae mech
        SmartDashboard.putString("Algae State", thisRobot.algae.algaeState.name());
        SmartDashboard.putNumber("Algae Current", thisRobot.algae.algaeMotor1.getOutputCurrent());
        SmartDashboard.putNumber("Algae Motor Pos", thisRobot.algae.algaeMotor1.getEncoder().getPosition());
        SmartDashboard.putNumber("Algae Motor Goal Pos", thisRobot.algae.algaeController.getSetpoint());
    
        //climber mech
        SmartDashboard.putString("Climber State", thisRobot.climber.state.name());
        SmartDashboard.putNumber("Climber Current", thisRobot.climber.climberMotor1.getOutputCurrent());
        SmartDashboard.putNumber("Climber Motor Pos", thisRobot.climber.climberMotor1.getEncoder().getPosition());
        
        //drivebase
        SmartDashboard.putString("Path Name", thisRobot.drivebase.pathName);
    }
}
