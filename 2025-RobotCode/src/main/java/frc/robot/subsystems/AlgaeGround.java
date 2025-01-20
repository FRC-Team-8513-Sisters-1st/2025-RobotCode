package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.logic.Enums.AlgaeGroundStates;

public class AlgaeGround {
    Robot thisRobot;

    AlgaeGroundStates state = AlgaeGroundStates.stowed;

    public SparkMax algaeGroundMotor1 = new SparkMax(Settings.algaeGroundMotor1CANID, MotorType.kBrushless);
    public SparkMax algaeGroundMotor2 = new SparkMax(Settings.algaeGroundMotor2CANID, MotorType.kBrushless);

    public AlgaeGround(Robot thisRobotIn) {
        
        thisRobot = thisRobotIn;
    }

    public void setState( AlgaeGroundStates state) {
        this.state = state;
    }

    public void setMotorPower() {

    }
}