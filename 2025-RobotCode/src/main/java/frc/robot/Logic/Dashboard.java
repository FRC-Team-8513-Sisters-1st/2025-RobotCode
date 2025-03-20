package frc.robot.Logic;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Settings;

public class Dashboard {

    Robot thisRobot;
    public Field2d attackPoitnField2d = new Field2d();
    public Field2d pathPlannerGoalField2d = new Field2d();
    public Field2d otfGoalField2d = new Field2d();

    public Dashboard(Robot thisRobotIn) {

        thisRobot = thisRobotIn;

        // fields only need to be put once, they auto update when you update the pose
        SmartDashboard.putData("attackPointPose", attackPoitnField2d);
        SmartDashboard.putData("pathPlannerGoalPose", pathPlannerGoalField2d);
        SmartDashboard.putData("otfGoalPose", otfGoalField2d);
    }

    public void updateDashboard() {
        //overall
        SmartDashboard.putNumber("Time", Timer.getFPGATimestamp());
        SmartDashboard.putBoolean("isOnRed", thisRobot.onRedAlliance);

        //autos
        SmartDashboard.putNumber("AutoStep", thisRobot.autoController.autoStep);
        SmartDashboard.putBoolean("FirstAutoRun", thisRobot.autoController.firstAutoBeingRun);
        
        //teleop 
        SmartDashboard.putString("Operator Chosen Side Of Reef", thisRobot.teleopController.operatorChosenSideOfReef.name());
        SmartDashboard.putString("Operator Chosen Side Of Feeder", thisRobot.teleopController.feederCloseOrFar.name());
        SmartDashboard.putBoolean("AutoScoreTeleop", thisRobot.teleopController.teleopAutoScore);
        
        // elevator motor stats
        SmartDashboard.putNumber("elevator motor 1 power", thisRobot.elevator.elevatorMotor1.getAppliedOutput());
        SmartDashboard.putNumber("elevator motor 2 power", thisRobot.elevator.elevatorMotor2.getAppliedOutput());
        SmartDashboard.putNumber("elevator motor 1 pos", thisRobot.elevator.elevatorMotor1.getEncoder().getPosition());
        SmartDashboard.putNumber("elevator motor 2 pos", thisRobot.elevator.elevatorMotor2.getEncoder().getPosition());
        SmartDashboard.putNumber("elevatorGoalState", thisRobot.elevator.m_controller.getGoal().position);
        SmartDashboard.putNumber("elevatorPosition", thisRobot.elevator.elevatorMotor1.getEncoder().getPosition());
        SmartDashboard.putString("elevatorState", thisRobot.elevator.state.name());
        SmartDashboard.putBoolean("Auto Elevator On", thisRobot.elevator.autoElevatorOn);
        SmartDashboard.putBoolean("New At Goal",thisRobot.elevator.newElevatorAtSetpoint());
 

        // coral mech stats
        SmartDashboard.putNumber("coral motor pos", thisRobot.coral.coralMotor1.getEncoder().getPosition());
        SmartDashboard.putNumber("coralSensor", thisRobot.coral.coralMotor1.getAnalog().getVoltage());
        SmartDashboard.putString("coralState", thisRobot.coral.state.name());
        SmartDashboard.putNumber("coral motor velocity", thisRobot.coral.coralMotor1.getEncoder().getVelocity());
        SmartDashboard.putNumber("coral motor power", thisRobot.coral.coralPower);
        SmartDashboard.putNumber("coral goal velocity", thisRobot.coral.coralPower * Settings.coralPowerToVeloctyFactor);



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
        SmartDashboard.putBoolean("IsRobotInReefZone", thisRobot.drivebase.isRobotInReefZone());
        pathPlannerGoalField2d.setRobotPose(thisRobot.drivebase.otfGoalPose);
        SmartDashboard.putNumber("DB Velocity", thisRobot.drivebase.getRobotVelopcity());
        SmartDashboard.putNumber("Teleop Auto Score Counter", thisRobot.teleopController.autoScoreCounter);
    }
}
