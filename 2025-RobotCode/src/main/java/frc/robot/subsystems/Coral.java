package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.logic.Enums.CoralIntakeStates;

public class Coral {
    Robot thisRobot;

    CoralIntakeStates state = CoralIntakeStates.stationary;

    public SparkMax coralMotor1 = new SparkMax(Settings.coralMotor1CANID, MotorType.kBrushless);

    public Coral(Robot thisRobotIn) {

        thisRobot = thisRobotIn;
    }

    public void setState(CoralIntakeStates state) {
        this.state = state;
    }

    public void setMotorPower() {
        if (thisRobot.teleopController.operatorJoystick1.getRawButton(Settings.buttonId_CoralIntake)) {
            coralMotor1.set(1);
        } else if (thisRobot.teleopController.operatorJoystick1.getRawButton(Settings.buttonId_CoralIntake)) {
            coralMotor1.set(-1);
        } else {
            coralMotor1.set(0);
        }
    }

}