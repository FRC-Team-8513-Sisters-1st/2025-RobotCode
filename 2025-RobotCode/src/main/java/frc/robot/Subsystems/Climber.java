package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;

import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Logic.Enums.ClimberStates;

public class Climber {
    Robot thisRobot;

    public ClimberStates state = ClimberStates.stowed;

    public SparkMax climberMotor1 = new SparkMax(Settings.climberMotor1CANID, MotorType.kBrushless);

    public Climber(Robot thisRobotIn) {
        
        thisRobot = thisRobotIn;
    }

    public void setState( ClimberStates state) {
        this.state = state;
    }

    public void setMotorPower() {

        climberMotor1.set(thisRobot.teleopController.manualJoystick.getRawAxis(1));

    }
}