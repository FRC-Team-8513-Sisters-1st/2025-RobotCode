package frc.robot.logic;

import frc.robot.Settings;
import frc.robot.logic.Enums.FeederStation;
import frc.robot.logic.Enums.RobotStates;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Robot;

public class TeleopController {

    Robot thisRobot;
    public Joystick driverXboxController = new Joystick(Settings.driverJoystickPort);
    public Joystick manualJoystick = new Joystick(4);

    public Joystick operatorJoystick1 = new Joystick(Settings.operatorJoystick1Port);
    public Joystick operatorJoystick2 = new Joystick(Settings.operatorJoystick2Port);

    RobotStates robotState = RobotStates.driving;

    Pose2d coralScoreGoalPose = new Pose2d();
    RobotStates operatorGoalAlgaeReefLevel;

    Rotation2d goalHeading = new Rotation2d();

    SlewRateLimiter xfilter = new SlewRateLimiter(4);
    SlewRateLimiter yfilter = new SlewRateLimiter(4);
    SlewRateLimiter rfilter = new SlewRateLimiter(4);

    public TeleopController(Robot thisRobotIn) {
        thisRobot = thisRobotIn;
    }

    public void initTele(){
        State state = new State(thisRobot.elevator.elevatorMotor1.getEncoder().getPosition(), 0);
        thisRobot.elevator.m_controller.setGoal(state);
    }

    public void driveTele() {

        thisRobot.algae.setMotorPower();
        thisRobot.coral.setMotorPower();
        thisRobot.elevator.setMotorPower();
        thisRobot.climber.setMotorPower();

        if (driverXboxController.getRawButton(Settings.buttonId_resetOdo)) {
            //uncomment this when done testing odom
            //thisRobot.drivebase.swerveDrive.resetOdometry(new Pose2d(thisRobot.drivebase.swerveDrive.getPose().getX(),
            //    thisRobot.drivebase.swerveDrive.getPose().getY(), new Rotation2d()));
            thisRobot.drivebase.swerveDrive.resetOdometry(new Pose2d(2,2, new Rotation2d()));
            goalHeading = new Rotation2d();
        }

        double xSpeedJoystick = -driverXboxController.getRawAxis(Settings.forwardBackwardsAxis); // forward back
        if (xSpeedJoystick < Settings.joystickDeadband && xSpeedJoystick > -Settings.joystickDeadband) {
            xSpeedJoystick = 0;
        }
        xSpeedJoystick = xfilter.calculate(xSpeedJoystick);

        double ySpeedJoystick = -driverXboxController.getRawAxis(Settings.leftRightAxis); // left right
        if (ySpeedJoystick < Settings.joystickDeadband && ySpeedJoystick > -Settings.joystickDeadband) {
            ySpeedJoystick = 0;

        }
        ySpeedJoystick = yfilter.calculate(ySpeedJoystick);
        double rSpeedJoystick = -driverXboxController.getRawAxis(Settings.rotAxis); // left right 2 at home, 4 on xbox
        if (rSpeedJoystick < Settings.joystickDeadband && rSpeedJoystick > -Settings.joystickDeadband) {
            rSpeedJoystick = 0;

        }
        rSpeedJoystick = rfilter.calculate(rSpeedJoystick);
        // if we are on red, flip the joysticks
        if (thisRobot.onRedAlliance) {
            xSpeedJoystick = -xSpeedJoystick;
            ySpeedJoystick = -ySpeedJoystick;
        }

        // cube the joystick values for smoother control
        double xInput = Math.pow(xSpeedJoystick, 3);
        double yInput = Math.pow(ySpeedJoystick, 3);
        double rInput = Math.pow(rSpeedJoystick, 3);

        double xV = xInput * thisRobot.drivebase.swerveDrive.getMaximumChassisVelocity();
        double yV = yInput * thisRobot.drivebase.swerveDrive.getMaximumChassisVelocity();
        double rV = rInput * thisRobot.drivebase.swerveDrive.getMaximumChassisAngularVelocity();

        double jX = driverXboxController.getRawAxis(Settings.rightJoystickX);
        double jY = driverXboxController.getRawAxis(Settings.rightJoystickY);

        if (Settings.headingJoystickControls) {
            if (Math.sqrt(jX * jX + jY * jY) > 0.5) {
                goalHeading = new Rotation2d(jX, -jY);
                goalHeading = goalHeading.minus(Rotation2d.fromDegrees(90));
            }
            rV = Settings.rJoystickController.calculate(
                    thisRobot.drivebase.swerveDrive.getPose().getRotation().minus(goalHeading).getDegrees(), 0);
        }
        // setting Pose2d

        thisRobot.stateMachine.forceCoralAndAlgae();
        thisRobot.stateMachine.copilotSideOfReef();
        thisRobot.stateMachine.copilotLevelToScore();
        thisRobot.stateMachine.copilotCloseOrFar();
        thisRobot.stateMachine.forceCoralAndAlgae();
        thisRobot.stateMachine.operatorFeederStation();
        thisRobot.stateMachine.enumRobotState();

        double leftTriggerValue;
        double rightTriggerValue;

        leftTriggerValue = thisRobot.teleopController.driverXboxController
                .getRawAxis(Settings.axisId_LeftBranch);
        if (leftTriggerValue > Settings.triggerDeadband) {
            thisRobot.coralReady2Score = true;
            setCoralScoreGoalPoseLeft();
        }

        rightTriggerValue = thisRobot.teleopController.driverXboxController
                .getRawAxis(Settings.axisId_RightBranch);
        if (rightTriggerValue > Settings.triggerDeadband) {
            if (operatorGoalAlgaeReefLevel == RobotStates.algaeIntakeL2) {
                thisRobot.algaeReady2Score = true;
            } else if (operatorGoalAlgaeReefLevel == RobotStates.algaeIntakeL3) {
                thisRobot.algaeReady2Score = true;
            } else {
                thisRobot.coralReady2Score = true;
            }

            setCoralScoreGoalPoseRight();
        }

        // feeder stations
        if (thisRobot.teleopController.driverXboxController.getRawButton(Settings.buttonId_RightFeederSt) && thisRobot.stateMachine.feederCloseOrFar == FeederStation.Far) {
            thisRobot.drivebase.attackPoint(Settings.rightFarFeederStation, 3);
        } else if (thisRobot.teleopController.driverXboxController.getRawButton(Settings.buttonId_RightFeederSt) && thisRobot.stateMachine.feederCloseOrFar == FeederStation.Close) {
            thisRobot.drivebase.attackPoint(Settings.rightCloseFeederStation, 3);
        }
        if (thisRobot.teleopController.driverXboxController.getRawButton(Settings.buttonId_LeftFeederSt) && thisRobot.stateMachine.feederCloseOrFar == FeederStation.Far) {
            thisRobot.drivebase.attackPoint(Settings.leftFarFeederStation, 3);
        } else if (thisRobot.teleopController.driverXboxController.getRawButton(Settings.buttonId_LeftFeederSt) && thisRobot.stateMachine.feederCloseOrFar == FeederStation.Close) {
            thisRobot.drivebase.attackPoint(Settings.leftCloseFeederStation, 3);
        }

        //actually drive
        if(leftTriggerValue > Settings.triggerDeadband){
            thisRobot.drivebase.attackPoint(coralScoreGoalPose, leftTriggerValue * 3);
        } else if (rightTriggerValue > Settings.triggerDeadband) {
            thisRobot.drivebase.attackPoint(coralScoreGoalPose, rightTriggerValue * 3);
        } else {
            thisRobot.drivebase.drive(xV, yV, rV, false);
        }

    }

    public void setCoralScoreGoalPoseRight() {
        switch (thisRobot.stateMachine.operatorChosenSideOfReef) {
            case AB:
                coralScoreGoalPose = Settings.coralRightAB;
                break;
            case CD:
                coralScoreGoalPose = Settings.coralRightCD;
                break;
            case EF:
                coralScoreGoalPose = Settings.coralRightEF;
                break;
            case GH:
                coralScoreGoalPose = Settings.coralRightGH;
                break;
            case IJ:
                coralScoreGoalPose = Settings.coralRightIJ;
                break;
            case KL:
                coralScoreGoalPose = Settings.coralRightKL;
                break;
        }
    }

    public void setCoralScoreGoalPoseLeft() {
        switch (thisRobot.stateMachine.operatorChosenSideOfReef) {
            case AB:
                coralScoreGoalPose = Settings.coralLeftAB;
                break;
            case CD:
                coralScoreGoalPose = Settings.coralLeftCD;
                break;
            case EF:
                coralScoreGoalPose = Settings.coralLeftEF;
                break;
            case GH:
                coralScoreGoalPose = Settings.coralLeftGH;
                break;
            case IJ:
                coralScoreGoalPose = Settings.coralLeftIJ;
                break;
            case KL:
                coralScoreGoalPose = Settings.coralLeftKL;
                break;
        }
    }
}
