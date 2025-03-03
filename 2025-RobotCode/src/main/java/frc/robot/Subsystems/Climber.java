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

    public void setState(ClimberStates state) {
        this.state = state;
    }

    public void setMotorPower() {
        climberMotor1.set(thisRobot.teleopController.manualJoystick.getRawAxis(1));

        thisRobot.teleopController.climbUsingDriverController();

        switch (state) {
            case stowed:
            climberMotor1.getEncoder().setPosition(Settings.climberStowedPos);
                break;
            case armOut:
            climberMotor1.getEncoder().setPosition(Settings.climberArmOutPos);
                break;
            case climbing:
            climberMotor1.getEncoder().setPosition(Settings.climberClimbingPos);
            default:
                break;
        }
    }
}