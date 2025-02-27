package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Logic.Enums.CoralIntakeStates;
import frc.robot.Logic.Enums.ElevatorStates;

public class Elevator {
    Robot thisRobot;

    public ElevatorStates state = ElevatorStates.stowed;
    public SparkMax elevatorMotor1 = new SparkMax(Settings.elevatorMotor1CANID, MotorType.kBrushless);
    public SparkMax elevatorMotor2 = new SparkMax(Settings.elevatorMotor2CANID, MotorType.kBrushless);
    public static int yAxisLeft = 1;
    public static int yAxisRight = 1;

    private static double elevatorMaxVelocity = 70;
    private static double elevatorMaxAcceleration = 90;
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

        // sets the elevator state if in reef zone
        if (state == ElevatorStates.L1 && autoElevatorOn) {
            State elevatorGoalPIDState = new State(Settings.elevatorPosL1, 0);
            m_controller.setGoal(elevatorGoalPIDState);
        }
        if (elevatorSafeToGo(thisRobot.drivebase.apGoalPose) && state == ElevatorStates.L2 && autoElevatorOn) {
            State elevatorGoalPIDState = new State(Settings.elevatorPosL2, 0);
            m_controller.setGoal(elevatorGoalPIDState);
        }
        if (elevatorSafeToGo(thisRobot.drivebase.apGoalPose) && state == ElevatorStates.L3 && autoElevatorOn) {
            State elevatorGoalPIDState = new State(Settings.elevatorPosL3, 0);
            m_controller.setGoal(elevatorGoalPIDState);
        }
        if (elevatorSafeToGo(thisRobot.drivebase.apGoalPose) && state == ElevatorStates.L4 && autoElevatorOn) {
            State elevatorGoalPIDState = new State(Settings.elevatorPosL4, 0);
            m_controller.setGoal(elevatorGoalPIDState);
        }
        // algae state if in reef zone
        if (elevatorSafeToGo(thisRobot.drivebase.apGoalPose) && state == ElevatorStates.L2a && autoElevatorOn) {
            State elevatorGoalPIDState = new State(Settings.elevatorPosA2, 0);
            m_controller.setGoal(elevatorGoalPIDState);
        }
        if (elevatorSafeToGo(thisRobot.drivebase.apGoalPose) && state == ElevatorStates.L3a && autoElevatorOn) {
            State elevatorGoalPIDState = new State(Settings.elevatorPosA3, 0);
            m_controller.setGoal(elevatorGoalPIDState);
        }
        if (state == ElevatorStates.scoreProcessor) {
            State elevatorGoalPIDState = new State(Settings.elevatorPosProcessor, 0);
            m_controller.setGoal(elevatorGoalPIDState);
        }

        double yLeftValue = thisRobot.teleopController.manualJoystick.getRawAxis(5) * 0.2; // make this copilot elevator
                                                                                     // controller
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

    public boolean elevatorAtSetpoint(){
        return Math.abs(elevatorMotor1.getEncoder().getPosition() - m_controller.getGoal().position) < 0.25;
    }

    public boolean elevatorSafeToGo(Pose2d goalPose) {
        Pose2d currentPose = thisRobot.drivebase.swerveDrive.getPose();
        if (Settings.getDistanceBetweenTwoPoses(goalPose, currentPose) < Settings.elevatorSafeToGoThold && thisRobot.coral.state == CoralIntakeStates.stationary) {
            return true && thisRobot.drivebase.isRobotInReefZone();
        } else {
            return false;
        }
    }
}