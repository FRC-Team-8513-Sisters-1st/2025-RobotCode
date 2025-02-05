package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.logic.Enums.ElevatorStates;

public class Elevator {
    Robot thisRobot;

    ElevatorStates state = ElevatorStates.stowed;

    public SparkMax elevatorMotor1 = new SparkMax(Settings.elevatorMotor1CANID, MotorType.kBrushless);
    public SparkMax elevatorMotor2 = new SparkMax(Settings.elevatorMotor2CANID, MotorType.kBrushless);
    public static int yAxisLeft = 1;
    public static int yAxisRight = 1;

    private static double elevatorMaxVelocity = 1;
    private static double elevatorMaxAcceleration = 0.2;
    private static double elevatorP = 0.2;
    private static double elevatorI = 0.025;
    private static double elevatorD = 0.0;
    private static double elevatorDt = 0.02;

    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(elevatorMaxVelocity,
            elevatorMaxAcceleration);
    private final ProfiledPIDController m_controller = new ProfiledPIDController(elevatorP, elevatorI, elevatorD, m_constraints, elevatorDt);

    public Elevator(Robot thisRobotIn) {

        thisRobot = thisRobotIn;
    }

    public void setState(ElevatorStates state) {
        this.state = state;
    }

    public void setMotorPower() {
        double yLeftValue = thisRobot.teleopController.driverXboxController.getRawAxis(yAxisLeft) * 0.2;
        if (Math.abs(yLeftValue) > 0.02) {
            elevatorMotor1.set(yLeftValue);
            elevatorMotor2.set(-yLeftValue);
            State state  = new State(elevatorMotor1.getEncoder().getPosition(), 0);
            m_controller.setGoal(state);
        } else {
            double power = m_controller.calculate(elevatorMotor1.getEncoder().getPosition());
            elevatorMotor1.set(power);
            elevatorMotor2.set(-power);
        }

        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral1)) {
            State state  = new State(Settings.elevatorPosL1, 0);
            m_controller.setGoal(state);
        }
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral2)) {
            State state  = new State(Settings.elevatorPosL2, 0);
            m_controller.setGoal(state);
        }
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral3)) {
            State state  = new State(Settings.elevatorPosL3, 0);
            m_controller.setGoal(state);
        }
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral4)) {
            State state  = new State(Settings.elevatorPosL4, 0);
            m_controller.setGoal(state);
        }

    }
}