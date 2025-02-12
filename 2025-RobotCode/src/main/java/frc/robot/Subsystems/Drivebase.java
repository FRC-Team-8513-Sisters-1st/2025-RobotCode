package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Robot;
import frc.robot.Settings;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
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
import com.pathplanner.lib.pathfinding.Pathfinding;

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
    String pathName = "";
    double elapsedTime;
    double timePathStarted;
    PathPlannerTrajectoryState trajGoalState;
    Field2d trajGoalPosition = new Field2d();

    LocalADStar pathGen = new LocalADStar();
    Rotation2d trajGoalRotation = new Rotation2d();
    PathConstraints constraints = new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));
    boolean otfReady = false;

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
        Pathfinding.ensureInitialized();
    }

    public void drive(double vx, double vy, double vr, boolean fieldCentric) {
        if (fieldCentric) {
            thisRobot.drivebase.swerveDrive.driveFieldOriented(new ChassisSpeeds(vx, vy, vr));
        } else {
            thisRobot.drivebase.swerveDrive.drive(new ChassisSpeeds(vx, vy, vr));
        }
    }

    public void initPath(String pathNameIn) {
        // create a path object from a path file
        pathName = pathNameIn;

        try {
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (FileVersionException | IOException | ParseException e) {
            System.out.println("Error in loading path");
            e.printStackTrace();
        }

        // flip if you are on red
        if (thisRobot.onRedAlliance) {
            path = path.flipPath();
        }

        // turn that path into a trajectory object
        try {
            traj = path.getIdealTrajectory(RobotConfig.fromGUISettings()).get();
        } catch (IOException | ParseException e) {
            System.out.println("Error in trajectory generation");
            e.printStackTrace();
        }

        // if we are a simulation set the robots pose to the starting pose of the path
        if (Robot.isSimulation()) {
            swerveDrive.resetOdometry(path.getStartingHolonomicPose().get());
        }

        // update a variable to keep track of the fact we loaded a new path but have not
        // begun to follow it
        loadedPathHasStarted = false;

    }

    public boolean followLoadedPath() {
        // returns true if path is over
        if (!loadedPathHasStarted) {
            timePathStarted = Timer.getFPGATimestamp();
            loadedPathHasStarted = true;
        }
        // get the elapsed time, currentTime - timePathStarted
        elapsedTime = Timer.getFPGATimestamp() - timePathStarted;

        // sample the trajctory for the current goal state
        if (elapsedTime > traj.getTotalTimeSeconds()) {
            return true;
        } else {
            // get the goal chasisSpeeds from the trajectory and tell the robot to drive at
            // that speed
            trajGoalState = traj.sample(elapsedTime);
            swerveDrive.driveFieldOriented(trajGoalState.fieldSpeeds);

            trajGoalPosition.setRobotPose(trajGoalState.pose);
            SmartDashboard.putData("path planner goal postition", trajGoalPosition);

            double dvx = Settings.xController.calculate(swerveDrive.getPose().getX(), trajGoalState.pose.getX());
            double dvy = Settings.yController.calculate(swerveDrive.getPose().getY(), trajGoalState.pose.getY());
            double dvr = Settings.rController.calculate(
                    swerveDrive.getPose().getRotation().minus(trajGoalState.pose.getRotation()).getDegrees(), 0);

            swerveDrive.driveFieldOriented(trajGoalState.fieldSpeeds.plus(new ChassisSpeeds(dvx, dvy, dvr)));

            return false;

        }

    }

    public boolean attackPoint(Pose2d goalPose) {
        double poseX = Settings.xController.calculate(swerveDrive.getPose().getX(), goalPose.getX());
        double poseY = Settings.yController.calculate(swerveDrive.getPose().getY(), goalPose.getY());
        double poseR = Settings.rController
                .calculate(swerveDrive.getPose().getRotation().minus(goalPose.getRotation()).getDegrees(), 0);

        swerveDrive.driveFieldOriented(new ChassisSpeeds(poseX, poseY, poseR));

        return Settings.getDistanceBetweenTwoPoses(goalPose, swerveDrive.getPose()) < Settings.coralScoreThold;

    }

    public void matchSimulatedOdomToPose() {
        swerveDrive.addVisionMeasurement(swerveDrive.getSimulationDriveTrainPose().get(), Timer.getFPGATimestamp());
    }

    public void initPathToPoint(Pose2d goalPose) {
        pathGen.setGoalPosition(goalPose.getTranslation());
        pathGen.setStartPosition(swerveDrive.getPose().getTranslation());

        trajGoalRotation = goalPose.getRotation();
        otfReady = false;
    }

    public void followOTFPath() {
        if (pathGen.isNewPathAvailable()) {
            GoalEndState ges = new GoalEndState(0, trajGoalRotation);
            path = pathGen.getCurrentPath(constraints, ges);
            if (path != null) {
                try {
                    traj = path.generateTrajectory(swerveDrive.getRobotVelocity(), swerveDrive.getPose().getRotation(),
                            RobotConfig.fromGUISettings());
                } catch (IOException | ParseException e) {
                    System.out.println("Error in trajectory generation");
                    e.printStackTrace();
                }
                // update a variable to keep track of the fact we loaded a new path but have not
                // begun to follow it
                loadedPathHasStarted = false;
                otfReady = true;
            }
        }
        if (otfReady)
            followLoadedPath();
    }

}