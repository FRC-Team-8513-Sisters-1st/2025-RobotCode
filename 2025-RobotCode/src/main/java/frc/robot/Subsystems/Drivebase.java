package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Second;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Robot;
import frc.robot.Settings;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivebase {
    // public vars
    Robot thisRobot;
    public SwerveDrive swerveDrive;

    // OTF/Path variables
    PathPlannerTrajectory traj;
    boolean loadedPathHasStarted = false;
    PathPlannerPath path;
    public String pathName = "";
    double elapsedTime;
    double timePathStarted;
    PathPlannerTrajectoryState trajGoalState;
    Field2d trajGoalPosition = new Field2d();
    double otfEndVelocity = 0.3;

    public PathPlannerPath pathPlannerGoalPose;
    public Pose2d newTeleopGoalPose = new Pose2d();

    // OTF Path Variables
    public Pathfinder generatePath = new LocalADStar();
    Rotation2d trajGoalRotation = new Rotation2d();
    PathConstraints oTFConstraints;
    boolean otfReady = false;
    public Pose2d otfGoalPose = new Pose2d();
    int nullPathCount = 0;
    boolean otfPathDone = true;
    boolean skipOTF = false;

    // ap variables
    public Pose2d apGoalPose = new Pose2d();
    boolean apDone = false;

    public Drivebase(Robot thisRobotIn) {
        double maximumSpeed = Units.feetToMeters(Settings.drivebaseMaxVelocityFPS);
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
        } catch (IOException e) {
            e.printStackTrace();
        }
        if(Robot.isReal()){
            swerveDrive.setCosineCompensator(true);
        }
        oTFConstraints = new PathConstraints(
            swerveDrive.getMaximumChassisVelocity(), 4,
            Units.degreesToRadians(300), Units.degreesToRadians(360));
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        thisRobot = thisRobotIn;

        SmartDashboard.putData("Path Planner Goal Pose", trajGoalPosition);
    }

    public void drive(double vx, double vy, double vr, boolean fieldCentric) {
        if (fieldCentric) {
            thisRobot.drivebase.swerveDrive.driveFieldOriented(new ChassisSpeeds(vx, vy, vr));
        } else {
            thisRobot.drivebase.swerveDrive.drive(new ChassisSpeeds(vx, vy, vr));
        }
    }

    public void initPath(String pathNameIn) {
        pathName = pathNameIn;

        try {
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (FileVersionException | IOException | ParseException e) {
            System.out.println("Error in loading path");
            e.printStackTrace();
        }

        if (thisRobot.onRedAlliance) {
            path = path.flipPath();
        }

        try {
            traj = path.getIdealTrajectory(RobotConfig.fromGUISettings()).get();
        } catch (IOException | ParseException e) {
            System.out.println("Error in trajectory generation");
            e.printStackTrace();
        }

        if (Robot.isSimulation()) {
            swerveDrive.resetOdometry(path.getStartingHolonomicPose().get());
        }

        loadedPathHasStarted = false;

    }

    public boolean followLoadedPath() {
        if (!loadedPathHasStarted) {
            timePathStarted = Timer.getFPGATimestamp();
            loadedPathHasStarted = true;
        }

        elapsedTime = Timer.getFPGATimestamp() - timePathStarted;

        if (elapsedTime > traj.getTotalTimeSeconds()) {
            return true;
        } else {

            trajGoalState = traj.sample(elapsedTime);
            swerveDrive.driveFieldOriented(trajGoalState.fieldSpeeds);

            trajGoalPosition.setRobotPose(trajGoalState.pose);

            double dvx = Settings.xController.calculate(swerveDrive.getPose().getX(), trajGoalState.pose.getX());
            double dvy = Settings.yController.calculate(swerveDrive.getPose().getY(), trajGoalState.pose.getY());
            double dvr;
            Rotation2d faceReefRotation2d;
            double percentThroughPath = elapsedTime / traj.getTotalTime().in(Second);
            if (isPoseInReefZone(apGoalPose) && percentThroughPath < 0.85 && percentThroughPath > 0.1) {
                if (thisRobot.onRedAlliance) {
                    faceReefRotation2d = Settings.reefZoneRed.getTranslation()
                            .minus(swerveDrive.getPose().getTranslation()).getAngle();
                } else {
                    faceReefRotation2d = Settings.reefZoneBlue.getTranslation()
                            .minus(swerveDrive.getPose().getTranslation()).getAngle();
                }
                dvr = Settings.rController.calculate(
                        swerveDrive.getPose().getRotation().minus(faceReefRotation2d).getDegrees(), 0);

            } else {
                dvr = Settings.rController.calculate(
                        swerveDrive.getPose().getRotation().minus(trajGoalState.pose.getRotation()).getDegrees(), 0);

            }

            swerveDrive.driveFieldOriented(trajGoalState.fieldSpeeds.plus(new ChassisSpeeds(dvx, dvy, dvr)));

            return false;

        }
    }

    public boolean attackPoint(Pose2d goalPose, double maxSpeed) {
        if (thisRobot.onRedAlliance) {
            goalPose = flipPoseToRed(goalPose);
        }

        State goalXState = new State(goalPose.getX(), 0);
        State goalYState = new State(goalPose.getY(), 0);
        State goalRState = new State(0, 0);

        double xVelocity = Settings.xControllerAP.calculate(thisRobot.drivebase.swerveDrive.getPose().getX(),
                goalXState);
        double yVelocity = Settings.yControllerAP.calculate(thisRobot.drivebase.swerveDrive.getPose().getY(),
                goalYState);
        double rVelocity = Settings.rControllerAP
                .calculate(thisRobot.drivebase.swerveDrive.getPose().getRotation().minus(goalPose.getRotation())
                        .getDegrees(), goalRState);

        double oldMag = Math.sqrt(xVelocity * xVelocity + yVelocity * yVelocity);
        double newMag = clamp(oldMag, maxSpeed);

        xVelocity = xVelocity * newMag / oldMag;
        yVelocity = yVelocity * newMag / oldMag;
        swerveDrive.driveFieldOriented(new ChassisSpeeds(xVelocity, yVelocity, rVelocity));
        thisRobot.dashboard.attackPoitnField2d.setRobotPose(goalPose);
        return Settings.getDistanceBetweenTwoPoses(goalPose, swerveDrive.getPose()) < Settings.coralScoreThold 
                && goalPose.getRotation().minus(swerveDrive.getPose().getRotation()).getDegrees() < Settings.coralScoreDegThold;

    }

    public void matchSimulatedOdomToPose() {
        swerveDrive.addVisionMeasurement(swerveDrive.getSimulationDriveTrainPose().get(), Timer.getFPGATimestamp());
    }

    public double clamp(double value, double max) {
        if (value > 0 && value > max) {
            return max;
        }

        if (value < 0 && value < -max) {
            return -max;
        }

        return value;
    }

    public boolean isRobotInReefZone() {
        if (thisRobot.onRedAlliance) {
            double x = thisRobot.drivebase.swerveDrive.getPose().minus(Settings.reefZoneRed).getX();
            double y = thisRobot.drivebase.swerveDrive.getPose().minus(Settings.reefZoneRed).getY();
            double distance = Math.sqrt(x * x + y * y);
            return distance < Settings.minDistanceFromReefZoneMeter;
        } else {
            double x = thisRobot.drivebase.swerveDrive.getPose().minus(Settings.reefZoneBlue).getX();
            double y = thisRobot.drivebase.swerveDrive.getPose().minus(Settings.reefZoneBlue).getY();
            double distance = Math.sqrt(x * x + y * y);
            return distance < Settings.minDistanceFromReefZoneMeter;
        }
    }

    public boolean isPoseInReefZone(Pose2d pose) {
        if (thisRobot.onRedAlliance) {
            double x = pose.minus(Settings.reefZoneRed).getX();
            double y = pose.minus(Settings.reefZoneRed).getY();
            double distance = Math.sqrt(x * x + y * y);
            return distance < Settings.minDistanceFromReefZoneMeter;
        } else {
            double x = pose.minus(Settings.reefZoneBlue).getX();
            double y = pose.minus(Settings.reefZoneBlue).getY();
            double distance = Math.sqrt(x * x + y * y);
            return distance < Settings.minDistanceFromReefZoneMeter;
        }

    }

    public Pose2d flipPoseToRed(Pose2d goalPose) {
        double goalXPos = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getFieldLength()
                - goalPose.getX();
        double goalYPos = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getFieldWidth()
                - goalPose.getY();
        Rotation2d goalRPos = goalPose.getRotation().rotateBy(new Rotation2d(Math.PI));

        if (goalXPos <= AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getFieldLength() / 2) {
            return goalPose;
        }

        return new Pose2d(goalXPos, goalYPos, goalRPos);
    }

    public void resetAPPIDControllers(Pose2d goalPose) {
        if (thisRobot.onRedAlliance) {
            goalPose = thisRobot.drivebase.flipPoseToRed(goalPose);
        }
        State goalXState = new State(thisRobot.drivebase.swerveDrive.getPose().getX(),
                thisRobot.drivebase.swerveDrive.getFieldVelocity().vxMetersPerSecond);
        State goalYState = new State(thisRobot.drivebase.swerveDrive.getPose().getY(),
                thisRobot.drivebase.swerveDrive.getFieldVelocity().vyMetersPerSecond);

        Settings.xControllerAP.reset(goalXState);
        Settings.yControllerAP.reset(goalYState);
        Settings.rControllerAP.reset(new State(0, 0));
    }

    public void initPathToPoint(Pose2d goalPose) {
        if (thisRobot.onRedAlliance) {
            goalPose = flipPoseToRed(goalPose);
        }
        Settings.xController.reset();
        Settings.yController.reset();
        Settings.rController.reset();
        generatePath.setGoalPosition(goalPose.getTranslation());
        // drive back if not going to reef zone and if in reef zone

        if (robotShouldBackupBeforeOTFPath(goalPose)) {
            generatePath.setStartPosition(
                    swerveDrive.getPose().transformBy(new Transform2d(-0.5, 0, new Rotation2d())).getTranslation());
        } else {
            generatePath.setStartPosition(swerveDrive.getPose().getTranslation());
        }

        trajGoalRotation = goalPose.getRotation();
        otfReady = false;
        otfPathDone = false;
        otfGoalPose = new Pose2d(goalPose.getX(), goalPose.getY(), trajGoalRotation);
    }

    public boolean followOTFPath() {
        if (generatePath.isNewPathAvailable()) {
            GoalEndState ges = new GoalEndState(otfEndVelocity, trajGoalRotation);
            path = generatePath.getCurrentPath(oTFConstraints, ges);

            if (path != null) {
                try {
                    // if we started pose with a backup we need to isert a waypoint where we
                    // actually start
                    List<Waypoint> wpList = path.getWaypoints();
                    if (robotShouldBackupBeforeOTFPath(otfGoalPose)) {

                        wpList.add(0,
                                new Waypoint(
                                        null,
                                        swerveDrive.getPose().getTranslation(),
                                        wpList.get(0).anchor()));

                        wpList.set(1, new Waypoint(wpList.get(0).anchor(), wpList.get(1).anchor(),
                                wpList.get(1).nextControl()));

                        path = new PathPlannerPath(wpList, oTFConstraints, null, ges);
                    }

                    traj = path.generateTrajectory(swerveDrive.getRobotVelocity(), swerveDrive.getPose().getRotation(),
                            RobotConfig.fromGUISettings());
                    thisRobot.dashboard.otfGoalField2d.getObject("traj").setTrajectory(ppTrajToWPITraj(traj));
                } catch (IOException | ParseException e) {
                    System.out.println("Error in trajectory generation");
                    e.printStackTrace();
                }

                loadedPathHasStarted = false;
                otfReady = true;
                nullPathCount = 0;
            } else {
                nullPathCount++;
            }
        }
        if (nullPathCount > 3) {
            otfPathDone = true;
            return true;
        }

        if (otfReady) {
            boolean pathState = followLoadedPath();
            if (pathState) {
                otfPathDone = true;
                return true;
            }
        } else {
            nullPathCount++;
        }
        return false;
    }

    public void initAstarAndAP(Pose2d otfPose, Pose2d apPose) {
        if (thisRobot.onRedAlliance) {
            otfPose = flipPoseToRed(otfPose);
            apPose = flipPoseToRed(apPose);
        }
        thisRobot.dashboard.otfGoalField2d.setRobotPose(apPose);
        skipOTF = false;
        if (Settings.getDistanceBetweenTwoPoses(apPose, swerveDrive.getPose()) < 0.75) {
            skipOTF = true;
            resetAPPIDControllers(apGoalPose);
        }
        apGoalPose = new Pose2d(apPose.getX(), apPose.getY(), apPose.getRotation());
        apDone = false;
        initPathToPoint(otfPose);
    }

    public boolean fromOTFSwitchToAP() {
        boolean pathDone = followOTFPath();
        if (pathDone || skipOTF) {
            apDone = thisRobot.drivebase.attackPoint(apGoalPose, 2) && pathDone;
            return apDone;

        } else {
            resetAPPIDControllers(apGoalPose);
        }
        return false;
    }

    public double getRobotVelopcity() {
        double velocityX = thisRobot.drivebase.swerveDrive.getFieldVelocity().vxMetersPerSecond;
        double velocityY = thisRobot.drivebase.swerveDrive.getFieldVelocity().vxMetersPerSecond;
        double velocity = Math.sqrt(velocityX * velocityX + velocityY * velocityY);

        return velocity;

    }

    public Trajectory ppTrajToWPITraj(PathPlannerTrajectory traj) {
        List<PathPlannerTrajectoryState> stateList = traj.getStates();
        List<Trajectory.State> wpiStateLists = new ArrayList<Trajectory.State>();
        for (PathPlannerTrajectoryState state : stateList) {
            Trajectory.State thisWPIState = new Trajectory.State(state.timeSeconds,
                    state.linearVelocity,
                    0,
                    state.pose,
                    0);
            wpiStateLists.add(thisWPIState);
        }
        return new Trajectory(wpiStateLists);
    }

    public boolean robotShouldBackupBeforeOTFPath(Pose2d goalPose) {
        if (thisRobot.onRedAlliance) {
            return (isPoseInReefZone(goalPose) == false && isRobotInReefZone())
                    || Settings.getDistanceBetweenTwoPoses(flipPoseToRed(Settings.processorAP),
                            swerveDrive.getPose()) < 1;
        } else {
            return (isPoseInReefZone(goalPose) == false && isRobotInReefZone())
                    || Settings.getDistanceBetweenTwoPoses(Settings.processorAP,
                            swerveDrive.getPose()) < 1;

        }

    }

}