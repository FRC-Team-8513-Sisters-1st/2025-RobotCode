package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivebase {
    // public vars
    Robot thisRobot;
    public SwerveDrive swerveDrive;

    // class variables
    PathPlannerTrajectory traj;
    boolean loadedPathHasStarted = false;
    PathPlannerPath path;
    public String pathName = "";
    double elapsedTime;
    double timePathStarted;
    PathPlannerTrajectoryState trajGoalState;
    Field2d trajGoalPosition = new Field2d();
    public PathPlannerPath pathPlannerGoalPose;

    Pathfinder generatePath = new LocalADStar();
    Rotation2d trajGoalRotation = new Rotation2d();
    PathConstraints oTFConstraints = new PathConstraints(
            Settings.maxVelocityAP, Settings.maxAccelerationAP,
            Units.degreesToRadians(360), Units.degreesToRadians(360));
    boolean otfReady = false;
    public Pose2d otfGoalPose = new Pose2d();

    public Drivebase(Robot thisRobotIn) {
        double maximumSpeed = Units.feetToMeters(Settings.drivebaseMaxVelocityFPS);
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
        } catch (IOException e) {
            e.printStackTrace();
        }
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
            double dvr = Settings.rController.calculate(
                    swerveDrive.getPose().getRotation().minus(trajGoalState.pose.getRotation()).getDegrees(), 0);

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
        return Settings.getDistanceBetweenTwoPoses(goalPose, swerveDrive.getPose()) < Settings.coralScoreThold;

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

    public Pose2d flipPoseToRed(Pose2d goalPose) {
        double goalXPos = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getFieldLength()
                - goalPose.getX();
        double goalYPos = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getFieldWidth()
                - goalPose.getY();
        Rotation2d goalRPos = goalPose.getRotation().rotateBy(new Rotation2d(Math.PI));

        if (goalXPos <= AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getFieldLength() / 2) {
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
        generatePath.setGoalPosition(goalPose.getTranslation());
        generatePath.setStartPosition(swerveDrive.getPose().getTranslation());

        trajGoalRotation = goalPose.getRotation();
        otfReady = false;
        otfGoalPose = new Pose2d(goalPose.getX(), goalPose.getY(), trajGoalRotation);
    }

    public boolean followOTFPath() {
        if (generatePath.isNewPathAvailable()) {
            GoalEndState ges = new GoalEndState(0, trajGoalRotation);
            path = generatePath.getCurrentPath(oTFConstraints, ges);
            if (path != null) {
                try {
                    traj = path.generateTrajectory(swerveDrive.getRobotVelocity(), swerveDrive.getPose().getRotation(),
                            RobotConfig.fromGUISettings());
                } catch (IOException | ParseException e) {
                    System.out.println("Error in trajectory generation");
                    e.printStackTrace();
                }

                loadedPathHasStarted = false;
                otfReady = true;
            }
        }

        if (otfReady)
            return followLoadedPath();
        return false;
    }

    public void fromOTFSwitchToAP() {
        if (Settings.getDistanceBetweenTwoPoses(thisRobot.drivebase.swerveDrive.getPose(), otfGoalPose) < Settings.otfToAPThold) {
            thisRobot.drivebase.attackPoint(otfGoalPose, 3);
        } else {
            followOTFPath();
            resetAPPIDControllers(otfGoalPose);
        }
    }
}