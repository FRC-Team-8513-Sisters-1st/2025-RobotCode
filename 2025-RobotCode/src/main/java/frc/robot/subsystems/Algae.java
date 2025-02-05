package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.logic.Enums.AlgaeIntakeStates;

public class Algae {
    Robot thisRobot;

    AlgaeIntakeStates algaeState = AlgaeIntakeStates.stationary;
    public SparkMax algaeMotor1 = new SparkMax(Settings.algaeMotor1CANID, MotorType.kBrushless);
    public SparkMax algaeMotor2 = new SparkMax(Settings.algaeMotor2CANID, MotorType.kBrushless);

    public static int algaeYAxisLeft = 1;
    public static int algaeXAxisLeft = 1;
    public static int algaeRAxisLeft = 1;
    public static int algaeYAxisRight = 1;
    public static int algaeXAxisRight = 1;
    public static int algaeRAxisRight = 1;
    int currentLimit = 4;
    int currentLimitBrokenCount = 0;

    PIDController algaeController = new PIDController(0.01, 0, 0);

    public Algae(Robot thisRobotIn) {

        thisRobot = thisRobotIn;

    }

    public void setState(AlgaeIntakeStates state) {
        this.algaeState = state;
    }

    public void setMotorPower() {
        switch (algaeState) {
            case intake:
                algaeController.setSetpoint(algaeMotor1.getEncoder().getPosition());
                algaeMotor1.set(0.5);
                break;
            case holdPosition:
                double power = algaeController.calculate(algaeMotor1.getEncoder().getPosition());
                algaeMotor1.set(power);
                currentLimitBrokenCount = 0;
                break;
            case outake:
                algaeMotor1.set(-1);
                break;
            case stationary:
                algaeMotor1.set(0);
                break;
        }
        if (algaeMotor1.getOutputCurrent() > currentLimit) {
            currentLimitBrokenCount++;
        }
        if (algaeState == AlgaeIntakeStates.intake && currentLimitBrokenCount > 50) {
            algaeState = AlgaeIntakeStates.holdPosition;
        }

        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(1)) {
            // not sure how to switch to next enum
            if (algaeState == AlgaeIntakeStates.stationary) {
                algaeState = AlgaeIntakeStates.intake;
            }
        }
    }

}