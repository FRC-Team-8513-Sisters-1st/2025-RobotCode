package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;

import com.revrobotics.spark.SparkMax;

import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.logic.Enums.ClimberStates;

public class Climber {
    Robot thisRobot;

    ClimberStates state = ClimberStates.stowed;

    public SparkMax climberMotor1 = new SparkMax(Settings.climberMotor1CANID, MotorType.kBrushless);

    Joystick tempJoystick = new Joystick(4);

    public Climber(Robot thisRobotIn) {
        
        thisRobot = thisRobotIn;
    }

    public void setState( ClimberStates state) {
        this.state = state;
    }

    public void setMotorPower() {

        climberMotor1.set(tempJoystick.getRawAxis(1));

    }
}