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
    public int storedElevatorState = 0; // 0 - stowed, 1 - L2, 2 - L3, 3 - L4
    public int storedAlgaeState = 2; // 2 - algae 2, 3 - algae 3
    public SparkMax elevatorMotor1 = new SparkMax(Settings.elevatorMotor1CANID, MotorType.kBrushless);
    public SparkMax elevatorMotor2 = new SparkMax(Settings.elevatorMotor2CANID, MotorType.kBrushless);
    public static int yAxisLeft = 1;
    public static int yAxisRight = 1;

    private static double elevatorMaxVelocity = 65;
    private static double elevatorMaxAcceleration = 50;
    private static double elevatorP = 0.2;
    private static double elevatorI = 0.025;
    private static double elevatorD = 0.0;
    private static double elevatorDt = 0.02;

    public boolean autoElevatorOn = true;

    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(elevatorMaxVelocity,
            elevatorMaxAcceleration);
    public final ProfiledPIDController m_controller = new ProfiledPIDController(elevatorP, elevatorI, elevatorD,
            m_constraints, elevatorDt);

    public Elevator(Robot thisRobotIn) {

        thisRobot = thisRobotIn;
        elevatorMotor1.getEncoder().setPosition(0);
    }

    public void setState(ElevatorStates state) {
        this.state = state;
    }

    public void setMotorPower() {

        // stores elevator state
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral1) && autoElevatorOn) {
            storedElevatorState = 0;
        }
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral2) && autoElevatorOn) {
            storedElevatorState = 1;
        }
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral3) && autoElevatorOn) {
            storedElevatorState = 2;
        }
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral4) && autoElevatorOn) {
            storedElevatorState = 3;
        }
        // algae
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Algae2) && autoElevatorOn) {
            storedAlgaeState = 2;
        }
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Algae3) && autoElevatorOn) {
            storedAlgaeState = 3;
        }


        // sets the elevator state if in reef zone
        if (thisRobot.stateMachine.isRobotInReefZone() == true && storedElevatorState == 0  && autoElevatorOn) {
            State elevatorGoalPIDState = new State(Settings.elevatorPosL1, 0);
            m_controller.setGoal(elevatorGoalPIDState);
        }
        if (thisRobot.stateMachine.isRobotInReefZone() == true && storedElevatorState == 1 && autoElevatorOn) {
            State elevatorGoalPIDState = new State(Settings.elevatorPosL2, 0);
            m_controller.setGoal(elevatorGoalPIDState);
        }
        if (thisRobot.stateMachine.isRobotInReefZone() == true &&storedElevatorState == 2 && autoElevatorOn) {
            State elevatorGoalPIDState = new State(Settings.elevatorPosL3, 0);
            m_controller.setGoal(elevatorGoalPIDState);
        }
        if (thisRobot.stateMachine.isRobotInReefZone() == true && storedElevatorState == 3 && autoElevatorOn) {
            State elevatorGoalPIDState = new State(Settings.elevatorPosL4, 0);
            m_controller.setGoal(elevatorGoalPIDState);
        }
        // algae state if in reef zone 
        if (thisRobot.stateMachine.isRobotInReefZone() == true && storedAlgaeState == 2  && autoElevatorOn) {
            State elevatorGoalPIDState = new State(Settings.elevatorPosA2, 0);
            m_controller.setGoal(elevatorGoalPIDState);
        }
        if (thisRobot.stateMachine.isRobotInReefZone() == true && storedAlgaeState == 3 && autoElevatorOn) {
            State elevatorGoalPIDState = new State(Settings.elevatorPosA3, 0);
            m_controller.setGoal(elevatorGoalPIDState);
        }

        // force elevator height
        if (thisRobot.teleopController.operatorJoystick2.getRawButton(Settings.buttonId_Drive) && thisRobot.teleopController.operatorJoystick1.getRawButton(Settings.buttonId_Coral1)) {
            State elevatorGoalPIDState = new State(Settings.elevatorPosL1, 0);
            m_controller.setGoal(elevatorGoalPIDState);
        }
        if (thisRobot.teleopController.operatorJoystick2.getRawButton(Settings.buttonId_Drive) && thisRobot.teleopController.operatorJoystick1.getRawButton(Settings.buttonId_Coral2)) {
            State elevatorGoalPIDState = new State(Settings.elevatorPosL2, 0);
            m_controller.setGoal(elevatorGoalPIDState);
        }
        if (thisRobot.teleopController.operatorJoystick2.getRawButton(Settings.buttonId_Drive) && thisRobot.teleopController.operatorJoystick1.getRawButton(Settings.buttonId_Coral3)) {
            State elevatorGoalPIDState = new State(Settings.elevatorPosL3, 0);
            m_controller.setGoal(elevatorGoalPIDState);
        }
        if (thisRobot.teleopController.operatorJoystick2.getRawButton(Settings.buttonId_Drive) && thisRobot.teleopController.operatorJoystick1.getRawButton(Settings.buttonId_Coral4)) {
            State elevatorGoalPIDState = new State(Settings.elevatorPosL4, 0);
            m_controller.setGoal(elevatorGoalPIDState);
        }
        // force to processor and all algae states
        if (thisRobot.teleopController.operatorJoystick2.getRawButton(Settings.buttonId_Drive) && thisRobot.teleopController.operatorJoystick1.getRawButton(Settings.buttonId_Algae2)) {
            State elevatorGoalPIDState = new State(Settings.elevatorPosA2, 0);
            m_controller.setGoal(elevatorGoalPIDState);
        }
        if (thisRobot.teleopController.operatorJoystick2.getRawButton(Settings.buttonId_Drive) && thisRobot.teleopController.operatorJoystick1.getRawButton(Settings.buttonId_Algae3)) {
            State elevatorGoalPIDState = new State(Settings.elevatorPosA3, 0);
            m_controller.setGoal(elevatorGoalPIDState);
        }

        double yLeftValue = thisRobot.teleopController.manualJoystick.getRawAxis(5); // make this copilot elevator controller
        if (Math.abs(yLeftValue) > 0.02) {
            elevatorMotor1.set(yLeftValue);
            elevatorMotor2.set(-yLeftValue);
            State state = new State(elevatorMotor1.getEncoder().getPosition(), 0);
            m_controller.setGoal(state);
            m_controller.reset(state);
        } else {
            double power = m_controller.calculate(elevatorMotor1.getEncoder().getPosition());
            elevatorMotor1.set(power);
            elevatorMotor2.set(-power);
        }
    }
}