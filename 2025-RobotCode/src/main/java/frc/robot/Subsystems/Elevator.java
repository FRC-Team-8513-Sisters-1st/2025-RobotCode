package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.logic.Enums.ElevatorStates;

public class Elevator {
    Robot thisRobot;

    ElevatorStates state = ElevatorStates.stowed;

    public SparkMax elevatorMotor1 = new SparkMax(Settings.elevatorMotor1CANID, MotorType.kBrushless);
    public SparkMax elevatorMotor2 = new SparkMax(Settings.elevatorMotor2CANID, MotorType.kBrushless);

    public Elevator(Robot thisRobotIn) {
        
        thisRobot = thisRobotIn;
    }

    public void setState( ElevatorStates state) {
        this.state = state;
    }

    public void setMotorPower() {

    }
}