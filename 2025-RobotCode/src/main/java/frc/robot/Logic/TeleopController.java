package frc.robot.logic;

import frc.robot.Settings;
import frc.robot.logic.Enums.CoralIntakeStates;
import frc.robot.logic.Enums.ElevatorStates;
import frc.robot.logic.Enums.FeederStation;
import frc.robot.logic.Enums.RobotStates;
import frc.robot.logic.Enums.SideOfReef;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Robot;

public class TeleopController {

    Robot thisRobot;
    public Joystick driverXboxController = new Joystick(Settings.driverJoystickPort);
    public Joystick manualJoystick = new Joystick(4);

    public Joystick operatorJoystick1 = new Joystick(Settings.operatorJoystick1Port);
    public Joystick operatorJoystick2 = new Joystick(Settings.operatorJoystick2Port);

    public FeederStation feederCloseOrFar = FeederStation.Close;
    public SideOfReef operatorChosenSideOfReef = SideOfReef.AB;

    RobotStates robotState = RobotStates.driving;

    public boolean firstOTFPath = false;
    public Pose2d teleopGoalPose = new Pose2d();
    public Pose2d teleopGoalPoseAstar = new Pose2d();

    Pose2d coralScoreGoalPose = new Pose2d();
    RobotStates operatorGoalAlgaeReefLevel;

    Rotation2d goalHeading = new Rotation2d();

    SlewRateLimiter xfilter = new SlewRateLimiter(4);
    SlewRateLimiter yfilter = new SlewRateLimiter(4);
    SlewRateLimiter rfilter = new SlewRateLimiter(4);

    public TeleopController(Robot thisRobotIn) {
        thisRobot = thisRobotIn;
    }

    public void initTele() {
        State state = new State(thisRobot.elevator.elevatorMotor1.getEncoder().getPosition(), 0);
        thisRobot.elevator.m_controller.reset(state);
        thisRobot.coral.coralController.setSetpoint(thisRobot.coral.coralMotor1.getEncoder().getPosition());
        thisRobot.coral.state = CoralIntakeStates.stationary;
        if(thisRobot.drivebase.swerveDrive.getPose().getX() == 0 && Robot.isSimulation()) {
            thisRobot.drivebase.swerveDrive.resetOdometry(new Pose2d(2,2, new Rotation2d()));
        }
    }

    public void driveTele() {

        updatChosenSideOfReefFromCopilot();
        elevatorSetLeve();
        forceCoralandAlgae();

        if (driverXboxController.getRawButton(Settings.buttonId_resetOdo)) {

            thisRobot.drivebase.swerveDrive.resetOdometry(new Pose2d(thisRobot.drivebase.swerveDrive.getPose().getX(),
                    thisRobot.drivebase.swerveDrive.getPose().getY(), new Rotation2d()));
            if(Robot.isSimulation()){
                if(thisRobot.onRedAlliance){
                    thisRobot.drivebase.swerveDrive.resetOdometry(thisRobot.drivebase.flipPoseToRed(new Pose2d(2, 2, new Rotation2d())));
                } else {
                    thisRobot.drivebase.swerveDrive.resetOdometry(new Pose2d(2, 2, new Rotation2d()));
                }
            }        
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

        double leftTriggerValue;
        double rightTriggerValue;

        leftTriggerValue = thisRobot.teleopController.driverXboxController
                .getRawAxis(Settings.axisId_LeftBranch);
        if (leftTriggerValue > Settings.triggerDeadband) {
            thisRobot.coralReady2Score = true;
            thisRobot.algaeReady2Score = false;
            setCoralScoreGoalPoseLeft();
        }

        rightTriggerValue = thisRobot.teleopController.driverXboxController
                .getRawAxis(Settings.axisId_RightBranch);
        if (rightTriggerValue > Settings.triggerDeadband) {
            if (operatorGoalAlgaeReefLevel == RobotStates.algaeIntakeL2) {
                thisRobot.algaeReady2Score = true;
                thisRobot.coralReady2Score = false;
            } else if (operatorGoalAlgaeReefLevel == RobotStates.algaeIntakeL3) {
                thisRobot.algaeReady2Score = true;
                thisRobot.coralReady2Score = false;
            } else {
                thisRobot.coralReady2Score = true;
                thisRobot.algaeReady2Score = false;
            }

            setCoralScoreGoalPoseRight();
        }

        readCopilotJoystickAndUdateCloseOrFar();
     
        boolean followPath = false;

        // actually drive + feeder st
        if (thisRobot.teleopController.driverXboxController.getRawButton(Settings.buttonId_RightFeederSt)
                && feederCloseOrFar == FeederStation.Far) {
            teleopGoalPose = Settings.rightFarFeederStationAP;
            teleopGoalPoseAstar = teleopGoalPose.transformBy(Settings.astarFeederStPoseOffset);
            followPath = true;
        } else if (thisRobot.teleopController.driverXboxController.getRawButton(Settings.buttonId_RightFeederSt)
                && feederCloseOrFar == FeederStation.Close) {
            teleopGoalPose = Settings.rightCloseFeederStationAP;
            teleopGoalPoseAstar = teleopGoalPose.transformBy(Settings.astarFeederStPoseOffset);
            followPath = true;
        } else if (thisRobot.teleopController.driverXboxController.getRawButton(Settings.buttonId_LeftFeederSt)
                && feederCloseOrFar == FeederStation.Far) {
            teleopGoalPose = Settings.leftFarFeederStationAP;
            teleopGoalPoseAstar = teleopGoalPose.transformBy(Settings.astarFeederStPoseOffset);
            followPath = true;
        } else if (thisRobot.teleopController.driverXboxController.getRawButton(Settings.buttonId_LeftFeederSt)
                && feederCloseOrFar == FeederStation.Close) {
            teleopGoalPose = Settings.leftCloseFeederStationAP;
            teleopGoalPoseAstar = teleopGoalPose.transformBy(Settings.astarFeederStPoseOffset);
            followPath = true;
        } else if (thisRobot.teleopController.driverXboxController.getRawButton(Settings.buttonId_processorAP)) {
            teleopGoalPose = Settings.processorAP;
            teleopGoalPoseAstar = teleopGoalPose.transformBy(Settings.astarProcesserPoseOffset);
            followPath = true;
        } else if (rightTriggerValue > Settings.triggerDeadband || leftTriggerValue > Settings.triggerDeadband) {
            followPath = true;
            teleopGoalPose = coralScoreGoalPose;
            teleopGoalPoseAstar = teleopGoalPose.transformBy(Settings.astarReefPoseOffset);
            if (Settings.getDistanceBetweenTwoPoses(thisRobot.drivebase.swerveDrive.getPose(), coralScoreGoalPose) < Settings.coralScoreThold && thisRobot.drivebase.getRobotVelopcity() < 0.04) {
                thisRobot.coral.state = CoralIntakeStates.outake;
            }
        } else{
            thisRobot.drivebase.drive(xV, yV, rV, true);
            firstOTFPath = true;
            thisRobot.drivebase.generatePath.setStartPosition(thisRobot.drivebase.swerveDrive.getPose().getTranslation());

        }

        if(followPath){
            if(firstOTFPath){
                thisRobot.drivebase.initAstarAndAP(teleopGoalPoseAstar, teleopGoalPose);
                firstOTFPath = false;
            }
            thisRobot.drivebase.fromOTFSwitchToAP();
        }

        thisRobot.algae.setMotorPower();
        thisRobot.coral.setMotorPower();
        thisRobot.elevator.setMotorPower();
        thisRobot.climber.setMotorPower();

        if (thisRobot.teleopController.manualJoystick.getRawButtonPressed(1)) {
            thisRobot.elevator.autoElevatorOn = !thisRobot.elevator.autoElevatorOn;
        }

    }

    public void setCoralScoreGoalPoseRight() {
        switch (operatorChosenSideOfReef) {
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
        switch (operatorChosenSideOfReef) {
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

    public void readCopilotJoystickAndUdateCloseOrFar() {
        if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_Far)) {
            feederCloseOrFar = FeederStation.Far;

        }
        if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_Close)) {
            feederCloseOrFar = FeederStation.Close;

        }
    }

    public void updatChosenSideOfReefFromCopilot() {
        if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_ab)) {
            operatorChosenSideOfReef = SideOfReef.AB;

        }
        if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_cd)) {
            operatorChosenSideOfReef = SideOfReef.CD;

        }
        if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_ef)) {
            operatorChosenSideOfReef = SideOfReef.EF;

        }
        if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_gh)) {
            operatorChosenSideOfReef = SideOfReef.GH;

        }
        if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_ij)) {
            operatorChosenSideOfReef = SideOfReef.IJ;

        }
        if (thisRobot.teleopController.operatorJoystick2.getRawButtonPressed(Settings.buttonId_kl)) {
            operatorChosenSideOfReef = SideOfReef.KL;

        }
    }

    public void elevatorSetLeve() {
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral1)) {
            thisRobot.elevator.state = ElevatorStates.L1;
        }
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral2)) {
            thisRobot.elevator.state = ElevatorStates.L2;
        }
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral3)) {
            thisRobot.elevator.state = ElevatorStates.L3;
        }
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Coral4)) {
            thisRobot.elevator.state = ElevatorStates.L4;
        }
        // algae
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Algae2)) {
            thisRobot.elevator.state = ElevatorStates.L2a;
        }
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_Algae3)) {
            thisRobot.elevator.state = ElevatorStates.L3a;
        }
        if (thisRobot.teleopController.operatorJoystick1.getRawButtonPressed(Settings.buttonId_processor)) {
            thisRobot.elevator.state = ElevatorStates.scoreProcessor;
        }
    }

    public void forceCoralandAlgae() {

        // force elevator height
        if (thisRobot.teleopController.operatorJoystick2.getRawButton(Settings.buttonId_forceElevator)
                && thisRobot.elevator.state == ElevatorStates.L1) {
            State elevatorGoalPIDState = new State(Settings.elevatorPosL1, 0);
            thisRobot.elevator.m_controller.setGoal(elevatorGoalPIDState);
        }
        if (thisRobot.teleopController.operatorJoystick2.getRawButton(Settings.buttonId_forceElevator)
                && thisRobot.elevator.state == ElevatorStates.L2) {
            State elevatorGoalPIDState = new State(Settings.elevatorPosL2, 0);
            thisRobot.elevator.m_controller.setGoal(elevatorGoalPIDState);
        }
        if (thisRobot.teleopController.operatorJoystick2.getRawButton(Settings.buttonId_forceElevator)
                && thisRobot.elevator.state == ElevatorStates.L3) {
            State elevatorGoalPIDState = new State(Settings.elevatorPosL3, 0);
            thisRobot.elevator.m_controller.setGoal(elevatorGoalPIDState);
        }
        if (thisRobot.teleopController.operatorJoystick2.getRawButton(Settings.buttonId_forceElevator)
                && thisRobot.elevator.state == ElevatorStates.L4) {
            State elevatorGoalPIDState = new State(Settings.elevatorPosL4, 0);
            thisRobot.elevator.m_controller.setGoal(elevatorGoalPIDState);
        }
        // force to processor and all algae states
        if (thisRobot.teleopController.operatorJoystick2.getRawButton(Settings.buttonId_forceElevator)
                && thisRobot.elevator.state == ElevatorStates.L2a) {
            State elevatorGoalPIDState = new State(Settings.elevatorPosA2, 0);
            thisRobot.elevator.m_controller.setGoal(elevatorGoalPIDState);
        }
        if (thisRobot.teleopController.operatorJoystick2.getRawButton(Settings.buttonId_forceElevator)
                && thisRobot.elevator.state == ElevatorStates.L3a) {
            State elevatorGoalPIDState = new State(Settings.elevatorPosA3, 0);
            thisRobot.elevator.m_controller.setGoal(elevatorGoalPIDState);
        }
        if (thisRobot.teleopController.operatorJoystick2.getRawButton(Settings.buttonId_processor)
                && thisRobot.elevator.state == ElevatorStates.scoreProcessor) {
            State elevatorGoalPIDState = new State(Settings.elevatorPosL2, 0);
            thisRobot.elevator.m_controller.setGoal(elevatorGoalPIDState);
        }
    }
}
