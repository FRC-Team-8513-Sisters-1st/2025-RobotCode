package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.spark.SparkMax;

import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Logic.Enums.CoralIntakeStates;
import frc.robot.Logic.Enums.ElevatorStates;

public class Coral {
    Robot thisRobot;

    public CoralIntakeStates state = CoralIntakeStates.stationary;

    public SparkMax coralMotor1 = new SparkMax(Settings.coralMotor1CANID, MotorType.kBrushless);
    public SparkMax funnelMotor1 = new SparkMax(Settings.funnelMotor1, MotorType.kBrushless);

    // sensor
    public PIDController coralController = new PIDController(.1, 0, 0);
    public boolean sensorFirstTime = false;
    public double holdCoralPos = 14;

    boolean seenBackEdge = false;

    public double currentBrokeTholdTime = 0;
    boolean sensorBrokeThold = false;

    public Coral(Robot thisRobotIn) {

        thisRobot = thisRobotIn;
        coralMotor1.getEncoder().setPosition(0);
    }

    public void setState(CoralIntakeStates state) {
        this.state = state;
    }

    public void setMotorPower() {

        switch (state) {
            case stationary:
                funnelMotor1.set(0);
                if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_CoralOutake)) {
                    state = CoralIntakeStates.outake;
                }
                double motorPower = coralController.calculate(coralMotor1.getEncoder().getPosition());
                coralMotor1.set(motorPower);

                if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_CoralIntake)) {
                    double lessenIntake = 1;
                    coralController.setSetpoint(coralController.getSetpoint() - lessenIntake);
                }

                break;
            case outake:
                double coralPower = 1;
                funnelMotor1.set(0.2);
                if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_CoralOutake)) {
                    state = CoralIntakeStates.stationary;
                    coralController.setSetpoint(coralMotor1.getEncoder().getPosition());
                }

                // sensor
                if ((coralMotor1.getAnalog().getVoltage() > Settings.sensorThold || sensorBrokeThold) && sensorFirstTime) {
                        sensorBrokeThold = true;
                        coralPower = 0.4;
                        if(coralMotor1.getAnalog().getVoltage() < Settings.sensorThold){
                            coralMotor1.getEncoder().setPosition(0);
                            coralController.setSetpoint(-2);
                            sensorFirstTime = false;
                            state = CoralIntakeStates.stationary;
                            sensorBrokeThold = false;
                        }
                } else if (coralMotor1.getAnalog().getVoltage() < Settings.sensorThold) {
                    sensorFirstTime = true;
                    sensorBrokeThold = false;
                }
                if(thisRobot.elevator.state == ElevatorStates.L1){
                    coralPower = 0.5;
                }
                coralMotor1.set(coralPower);
                break;
            default:
                break;
        }
    }
}
