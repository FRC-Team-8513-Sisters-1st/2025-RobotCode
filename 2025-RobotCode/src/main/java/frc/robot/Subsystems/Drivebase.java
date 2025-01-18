package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Robot;
import frc.robot.Settings;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;

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

    public Drivebase(Robot thisRobotIn) {
        double maximumSpeed = Units.feetToMeters(Settings.drivebaseMaxVelocityFPS);
        File swerveJsonDirectory= new File(Filesystem.getDeployDirectory(), "swerve");
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
        } catch (IOException e) {
            e.printStackTrace();
        }
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        thisRobot = thisRobotIn;
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
            double dvr = Settings.rController.calculate(swerveDrive.getPose().getRotation().minus(trajGoalState.pose.getRotation()).getDegrees(), 0);
            
            swerveDrive.driveFieldOriented(trajGoalState.fieldSpeeds.plus(new ChassisSpeeds(dvx, dvy, dvr)));

            return false;


        }

    }

}