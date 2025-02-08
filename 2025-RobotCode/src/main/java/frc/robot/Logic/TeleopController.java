package frc.robot.logic;

import frc.robot.Settings;
import frc.robot.logic.Enums.RobotStates;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Robot;

public class TeleopController {

    Robot thisRobot;
    public Joystick driverXboxController = new Joystick(Settings.driverJoystickPort);

    public Joystick operatorJoystick1 = new Joystick(Settings.operatorJoystick1Port);
    public Joystick operatorJoystick2 = new Joystick(Settings.operatorJoystick2Port);

    RobotStates robotState = RobotStates.driving;

    Pose2d coralScoreGoalPose = new Pose2d();
    RobotStates operatorGoalAlgaeReefLevel;

    public TeleopController(Robot thisRobotIn) {
        thisRobot = thisRobotIn;
    }

    public void driveTele() {

        thisRobot.algae.setMotorPower();
        thisRobot.coral.setMotorPower();
        thisRobot.elevator.setMotorPower();
        thisRobot.climber.setMotorPower();

        if (driverXboxController.getRawButton(Settings.buttonId_resetOdo)) {
            thisRobot.drivebase.swerveDrive.resetOdometry( new Pose2d(thisRobot.drivebase.swerveDrive.getPose().getX(), thisRobot.drivebase.swerveDrive.getPose().getY(), new Rotation2d()));
        }

        double xSpeedJoystick = -driverXboxController.getRawAxis(Settings.forwardBackwardsAxis); // forward back
        if (xSpeedJoystick < Settings.joystickDeadband && xSpeedJoystick > -Settings.joystickDeadband) {
            xSpeedJoystick = 0;
        }

        double ySpeedJoystick = -driverXboxController.getRawAxis(Settings.leftRightAxis); // left right
        if (ySpeedJoystick < Settings.joystickDeadband && ySpeedJoystick > -Settings.joystickDeadband) {
            ySpeedJoystick = 0;
        }

        double rSpeedJoystick = -driverXboxController.getRawAxis(Settings.rotAxis); // left right 2 at home, 4 on xbox
        if (rSpeedJoystick < Settings.joystickDeadband && rSpeedJoystick > -Settings.joystickDeadband) {
            rSpeedJoystick = 0;
        }

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

        if (thisRobot.teleopController.driverXboxController.getRawButton(Settings.buttonId_RightFeederSt)) {
            thisRobot.drivebase.attackPoint(Settings.rightCloseFeederStation);
        } else {
            thisRobot.drivebase.drive(xV, yV, rV, true);
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
