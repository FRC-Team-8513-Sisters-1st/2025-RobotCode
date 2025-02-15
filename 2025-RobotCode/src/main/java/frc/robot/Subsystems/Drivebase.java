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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
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
    PathPlannerPath pathPlannerGoalPose;

    Field2d targetField = new Field2d();
    Field2d scoreLeftField = new Field2d();
    Field2d scoreRightField = new Field2d();

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

    public boolean attackPoint(Pose2d goalPose, double maxSpeed) {
        if (thisRobot.onRedAlliance) {
            goalPose = flipPoseToRed(goalPose);
        }

        State goalXState = new State(goalPose.getX(), 0);
        State goalYState = new State(goalPose.getY(), 0);
        State goalRState = new State(0,0);

        double xVelocity = Settings.xControllerAP.calculate(thisRobot.drivebase.swerveDrive.getPose().getX(),
                goalXState);
        double yVelocity = Settings.yControllerAP.calculate(thisRobot.drivebase.swerveDrive.getPose().getY(),
                goalYState);
        double rVelocity = Settings.rControllerAP
                .calculate(thisRobot.drivebase.swerveDrive.getPose().getRotation().minus(goalPose.getRotation()).getDegrees(), goalRState);
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
                - goalPose.getX();;
        double goalYPos = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getFieldWidth()
                - goalPose.getY();
        Rotation2d goalRPos = goalPose.getRotation().rotateBy(new Rotation2d(Math.PI));

        return new Pose2d(goalXPos, goalYPos, goalRPos);
    }

    public void resetAPPIDControllers( Pose2d goalPose) {
        if (thisRobot.onRedAlliance) {
            goalPose = thisRobot.drivebase.flipPoseToRed(goalPose);
        }
        State goalXState = new State(thisRobot.drivebase.swerveDrive.getPose().getX(),
                thisRobot.drivebase.swerveDrive.getFieldVelocity().vxMetersPerSecond);
        State goalYState = new State(thisRobot.drivebase.swerveDrive.getPose().getY(),
                thisRobot.drivebase.swerveDrive.getFieldVelocity().vyMetersPerSecond);

        Settings.xControllerAP.reset(goalXState);
        Settings.yControllerAP.reset(goalYState);
        Settings.rControllerAP.reset(new State(0,0));
    }
}