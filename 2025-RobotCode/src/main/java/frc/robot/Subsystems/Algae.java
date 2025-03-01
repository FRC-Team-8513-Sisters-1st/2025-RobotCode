package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Logic.Enums.AlgaeIntakeStates;

public class Algae {
    Robot thisRobot;

    public AlgaeIntakeStates algaeState = AlgaeIntakeStates.stationary;
    public SparkMax algaeMotor1 = new SparkMax(Settings.algaeMotor1CANID, MotorType.kBrushless);

    int currentLimit = 4;
    int currentLimitBrokenCount = 0;

    double algaeOutTime = 0;
    boolean algaeOutFirstTime = true;

    int algaeSetState = 3;

    public PIDController algaeController = new PIDController(0.01, 0, 0);

    public Algae(Robot thisRobotIn) {

        thisRobot = thisRobotIn;

    }

    public void setState(AlgaeIntakeStates state) {
        this.algaeState = state;
    }

    public void setMotorPower() {
        if (thisRobot.teleopController.manualJoystick.getRawAxis(3) > Settings.triggerDeadband) {
            if (algaeState == AlgaeIntakeStates.intake) {
                algaeState = AlgaeIntakeStates.stationary;
            } else {
                algaeState = AlgaeIntakeStates.intake;
            }
        }
        if (thisRobot.teleopController.manualJoystick.getRawButtonPressed(6)) {
            algaeState = AlgaeIntakeStates.outake; 
        }

        if (algaeState == AlgaeIntakeStates.intake) {
            algaeController.setSetpoint(algaeMotor1.getEncoder().getPosition());
            algaeMotor1.set(0.75);
        }
        if (algaeState == AlgaeIntakeStates.holdPosition) {
            double power = algaeController.calculate(algaeMotor1.getEncoder().getPosition());
            algaeMotor1.set(power);
            currentLimitBrokenCount = 0;
        }
        if (algaeState == AlgaeIntakeStates.outake) {
            if (algaeOutFirstTime) {
                algaeOutTime = Timer.getFPGATimestamp();
                algaeOutFirstTime = false;
            }
            if (Timer.getFPGATimestamp() - algaeOutTime > 1) {
                algaeState = AlgaeIntakeStates.stationary;
            }
            algaeMotor1.set(-1);
        }
        if (algaeState == AlgaeIntakeStates.stationary) {
            algaeOutFirstTime = true;
            algaeMotor1.set(0);
        }

        if (algaeMotor1.getOutputCurrent() > currentLimit) {
            currentLimitBrokenCount++;
        }

        if (algaeState == AlgaeIntakeStates.intake && currentLimitBrokenCount > 35) {
            algaeState = AlgaeIntakeStates.holdPosition;
        }

        if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_AlgaeIntake)) {
            if (algaeState == AlgaeIntakeStates.intake) {
                algaeState = AlgaeIntakeStates.stationary;
            } else {
                algaeState = AlgaeIntakeStates.intake;
            }
        }

        if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_AlgaeOutake)) {
            algaeState = AlgaeIntakeStates.outake;
        }

        if (thisRobot.teleopController.manualJoystick.getRawAxis(2) > Settings.triggerDeadband) {
            
        }
    }
}