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
    public boolean sensorFirstTime = true;
    public double holdCoralPos = -2;

    public double currentBrokeTholdTime = 0;
    boolean sensorBrokeThold = false;
    boolean manualOutakePressed = false;

    public Coral(Robot thisRobotIn) {

        thisRobot = thisRobotIn;
        coralMotor1.getEncoder().setPosition(0);
    }

    public void setState(CoralIntakeStates state) {
        this.state = state;
    }

    public void setMotorPower() {

        if (thisRobot.teleopController.manualJoystick.getRawAxis(2) > Settings.triggerDeadband) {
            if (state == CoralIntakeStates.intake) {
                coralController.setSetpoint(coralMotor1.getEncoder().getPosition());
                state = CoralIntakeStates.stationary;
            } else {
                state = CoralIntakeStates.intake;
            }
        }
        if (thisRobot.teleopController.manualJoystick.getRawButtonPressed(5) && thisRobot.isTeleop()) {
            if (state == CoralIntakeStates.outake) {
                coralController.setSetpoint(coralMotor1.getEncoder().getPosition());
                state = CoralIntakeStates.stationary;
            } else {
                state = CoralIntakeStates.outake;
            }        }

        switch (state) {
            case stationary:
                funnelMotor1.set(0);
                if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_CoralOutake) && thisRobot.isTeleop()) {
                    state = CoralIntakeStates.outake;
                }
                double motorPower = coralController.calculate(coralMotor1.getEncoder().getPosition());
                coralMotor1.set(motorPower);

                // shift coral in and out when button pressed
                if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_CoralIntake) && thisRobot.isTeleop()) {
                    double lessenIntake = 1;
                    coralController.setSetpoint(coralController.getSetpoint() - lessenIntake);
                }
                if (thisRobot.teleopController.operatorJoystick1
                        .getRawButtonPressed(Settings.buttonId_CoralOutakeALittle)&& thisRobot.isTeleop()) {
                    double lessenIntake = -1;
                    coralController.setSetpoint(coralController.getSetpoint() - lessenIntake);
                }

                break;
            case outake:
                double coralPower = 0.8;
                funnelMotor1.set(0.2);
                if ((thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_CoralOutake) ||
                thisRobot.teleopController.manualJoystick.getRawButtonPressed(5)) && thisRobot.isTeleop()) {
                    state = CoralIntakeStates.stationary;
                    coralController.setSetpoint(coralMotor1.getEncoder().getPosition());
                }

                // if we see coral prep to stop when it passes sensor
                if ((coralMotor1.getAnalog().getVoltage() > Settings.sensorThold || sensorBrokeThold)
                        && sensorFirstTime) {
                    sensorBrokeThold = true;
                    // after we see coral slow down so it doesnt over shoot
                    coralPower = 0.2;
                    if (coralMotor1.getAnalog().getVoltage() < Settings.sensorThold) {
                        coralMotor1.getEncoder().setPosition(0);
                        coralController.setSetpoint(holdCoralPos);
                        sensorFirstTime = false;
                        state = CoralIntakeStates.stationary;
                        sensorBrokeThold = false;
                    }
                } else if (coralMotor1.getAnalog().getVoltage() < Settings.sensorThold) {
                    sensorFirstTime = true;
                    sensorBrokeThold = false;
                }
                // lower power for L1 so it doesnt bounce out
                if (thisRobot.elevator.state == ElevatorStates.L1 && thisRobot.drivebase.isRobotInReefZone()) {
                    coralPower = 0.3;
                }
                if (thisRobot.elevator.state == ElevatorStates.L4 && thisRobot.drivebase.isRobotInReefZone()) {
                    if (thisRobot.isAutonomous()){
                        coralPower = 0.65;
                    } else {
                        coralPower = 0.5;
                    }
                }
                if (thisRobot.elevator.state == ElevatorStates.L2 && thisRobot.drivebase.isRobotInReefZone()) {
                    if(thisRobot.isAutonomous()) {
                        coralPower = 0.8;
                    } else{
                        coralPower = 0.6;
                    }
                }
                if (thisRobot.elevator.state == ElevatorStates.L3 && thisRobot.drivebase.isRobotInReefZone()) {
                    coralPower = 0.6;
                }
                coralMotor1.set(coralPower);
                break;
            case intake: //we will never break out of this state unless intake is pressed again. maybe should add some escape conditions.
                coralController.setSetpoint(coralMotor1.getEncoder().getPosition());
                coralMotor1.set(-0.25);
                if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_CoralOutake)) {
                    state = CoralIntakeStates.stationary;
                }
            default:
                break;
        }
    }
}
