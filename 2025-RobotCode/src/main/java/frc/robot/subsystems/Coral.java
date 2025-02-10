package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.spark.SparkMax;

import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.logic.Enums.CoralIntakeStates;

public class Coral {
    Robot thisRobot;

    CoralIntakeStates state = CoralIntakeStates.stationary;

    public SparkMax coralMotor1 = new SparkMax(Settings.coralMotor1CANID, MotorType.kBrushless);

    public double currentBrokeTholdTime = 0;

    public boolean firstTime = true;

    public Coral(Robot thisRobotIn) {

        thisRobot = thisRobotIn;
    }

    public void setState(CoralIntakeStates state) {
        this.state = state;
    }

    public void setMotorPower() {
        if (thisRobot.teleopController.operatorJoystick1.getRawButton(Settings.buttonId_CoralIntake)) {
            coralMotor1.set(-0.25);
        } else if (thisRobot.teleopController.operatorJoystick1.getRawButton(Settings.buttonId_CoralOutake)) {
            coralMotor1.set(0.75);
        } else {
            coralMotor1.set(0);
        }

        switch (state) {
            case stationary:
                if (thisRobot.teleopController.operatorJoystick1.getRawButton(Settings.buttonId_CoralIntake)) {
                    coralMotor1.set(-0.25);
                } else {
                    coralMotor1.set(0);
                }
                if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_CoralOutake)) {
                    state = CoralIntakeStates.outake;
                }
                break;
            case outake:
            coralMotor1.set(0.75);
            if (coralMotor1.getOutputCurrent() > 8 && firstTime) {
                firstTime = false;
                currentBrokeTholdTime = Timer.getFPGATimestamp();
            } if (coralMotor1.getOutputCurrent() < 8) {
                firstTime = true;
                currentBrokeTholdTime = Timer.getFPGATimestamp();
            }

            if (Timer.getFPGATimestamp() - currentBrokeTholdTime > 0.12) {
                state = CoralIntakeStates.stationary;
            }

            if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_CoralOutake)) {
                state = CoralIntakeStates.stationary;
            }
            break;
            default:
                break;
        }
    }
}
