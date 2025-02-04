package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.logic.Enums.AlgaeIntakeStates;


public class Algae {
    Robot thisRobot;

    AlgaeIntakeStates state = AlgaeIntakeStates.stationary;

    public SparkMax algaeMotor1 = new SparkMax(Settings.algaeMotor1CANID, MotorType.kBrushless);
    public SparkMax algaeMotor2 = new SparkMax(Settings.algaeMotor2CANID, MotorType.kBrushless);

    public Algae(Robot thisRobotIn) {
        
        thisRobot = thisRobotIn;

    }
    
    public void setState( AlgaeIntakeStates state) {
        this.state = state;
    }

    public void setMotorPower() {

    }
}