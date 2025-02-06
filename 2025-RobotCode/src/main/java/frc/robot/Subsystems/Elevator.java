package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private static double elevatorP = 0.1;
    private static double elevatorI = 0.0;
    private static double elevatorD = 0.0;
    private static double elevatorDt = 0.02;

    Field2d targetField = new Field2d();
    Field2d scoreLeftField = new Field2d();
    Field2d scoreRightField = new Field2d();

    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(elevatorMaxVelocity,
            elevatorMaxAcceleration);
    private final PIDController m_controller = new PIDController(elevatorP, elevatorI, elevatorD);

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
            m_controller.setSetpoint(elevatorMotor1.getEncoder().getPosition());
        } else {
            double power = m_controller.calculate(elevatorMotor1.getEncoder().getPosition());
            SmartDashboard.putNumber("Power", power);
            if (power > 0 && power > 0.3) {
                power = 0.3;
            }
            if (power < 0 && power < -0.3) {
                power = -0.3;
            }
            elevatorMotor1.set(power);
            elevatorMotor2.set(-power);
        }

        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(1)) {
            m_controller.setSetpoint(2);
        }
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(2)) {
            m_controller.setSetpoint(4);
        }
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(3)) {
            m_controller.setSetpoint(6);
        }
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(4)) {
            m_controller.setSetpoint(8);
        }

    }

    public void updateOffsetPose(int target) {
        Pose2d tagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(target).get()
                .toPose2d();
        double dx = 1; // meters
        double yLeft = -0.25;
        double yRight = 0.1;
        targetField.setRobotPose(tagPose);
        SmartDashboard.putData("targetField", targetField);

        Transform2d tagToRobotScoreLeftTransform = new Transform2d(dx, yLeft, new Rotation2d(Math.PI));
        Transform2d tagToRobotScoreRightTransform = new Transform2d(dx, yRight, new Rotation2d(Math.PI));

        Pose2d scoreLeftGoalPose = tagPose.transformBy(tagToRobotScoreLeftTransform);
        scoreLeftField.setRobotPose(scoreLeftGoalPose);
        SmartDashboard.putData("leftGoalPose", scoreLeftField);

        tagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(target).get().toPose2d();
        Pose2d scoreRightGoalsPOse = tagPose.transformBy(tagToRobotScoreRightTransform);
        scoreRightField.setRobotPose(scoreRightGoalsPOse);
        SmartDashboard.putData("rightGoalPose", scoreRightField);
    }
}