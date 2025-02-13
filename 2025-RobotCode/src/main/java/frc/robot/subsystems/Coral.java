package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.spark.SparkMax;

import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.logic.Enums.CoralIntakeStates;

public class Coral {
    Robot thisRobot;

    CoralIntakeStates state = CoralIntakeStates.stationary;

    public SparkMax coralMotor1 = new SparkMax(Settings.coralMotor1CANID, MotorType.kBrushless);

    // sensor
    public PIDController coralController = new PIDController(.02, 0, 0);
    public boolean sensorFirstTime = true;
    public double holdCoralPos = 3.5;

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
                if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_CoralOutake)) {
                    state = CoralIntakeStates.outake;
                }
                double motorPower = coralController.calculate(coralMotor1.getEncoder().getPosition());
                coralMotor1.set(motorPower);
                if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_CoralIntake)) {
                    double lessenIntake = 0.5;
                    coralController.setSetpoint(coralController.getSetpoint() - lessenIntake);
                }

                break;
            case outake:
                coralMotor1.set(1);
                if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_CoralOutake)) {
                    state = CoralIntakeStates.stationary;
                    coralController.setSetpoint(coralMotor1.getEncoder().getPosition());
                }

                // sensor 
                if (coralMotor1.getAnalog().getVoltage() > Settings.sensorThold && sensorFirstTime) {
                    coralMotor1.getEncoder().setPosition(0);
                    coralController.setSetpoint(holdCoralPos);
                    sensorFirstTime = false;
                    state = CoralIntakeStates.stationary;
                } else if (coralMotor1.getAnalog().getVoltage() < Settings.sensorThold) {
                    sensorFirstTime = true;
                }
                break;
            default:
                break;
        }
    }
}
