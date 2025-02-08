package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.logic.Enums.AlgaeIntakeStates;

public class Algae {
    Robot thisRobot;

    AlgaeIntakeStates algaeState = AlgaeIntakeStates.stationary;
    public SparkMax algaeMotor1 = new SparkMax(Settings.algaeMotor1CANID, MotorType.kBrushless);

    public static int algaeYAxisLeft = 1;
    public static int algaeXAxisLeft = 1;
    public static int algaeRAxisLeft = 1;
    public static int algaeYAxisRight = 1;
    public static int algaeXAxisRight = 1;
    public static int algaeRAxisRight = 1;

    int currentLimit = 4;
    int currentLimitBrokenCount = 0;

    double algaeOutTime = 0;
    boolean algaeOutFirstTime = true;

    int algaeSetState = 3;

    PIDController algaeController = new PIDController(0.01, 0, 0);

    public Algae(Robot thisRobotIn) {

        thisRobot = thisRobotIn;

    }

    public void setState(AlgaeIntakeStates state) {
        this.algaeState = state;
    }

    public void setMotorPower() {
        if (algaeSetState == 0) {
            algaeController.setSetpoint(algaeMotor1.getEncoder().getPosition());
            algaeMotor1.set(0.75);
        }
        if (algaeSetState == 1) {
            double power = algaeController.calculate(algaeMotor1.getEncoder().getPosition());
            algaeMotor1.set(power);
            currentLimitBrokenCount = 0;
        }
        if (algaeSetState == 2) {
            if (algaeOutFirstTime) {
                algaeOutTime = Timer.getFPGATimestamp();
                algaeOutFirstTime = false;
            }
            if (Timer.getFPGATimestamp() - algaeOutTime > 1) {
                algaeSetState = 3;
            }
            algaeMotor1.set(-1);
        }
        if (algaeSetState == 3) {
            algaeOutFirstTime = true;
            algaeMotor1.set(0);
        }

        if (algaeMotor1.getOutputCurrent() > currentLimit) {
            currentLimitBrokenCount++;
        }

        if (algaeSetState == 0 && currentLimitBrokenCount > 35) {
            algaeSetState = 1;
        }

        if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_AlgaeIntake)){
            algaeSetState = 0;
        }

        if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_AlgaeOutake)){
            algaeSetState = 2;
        }

    }
}