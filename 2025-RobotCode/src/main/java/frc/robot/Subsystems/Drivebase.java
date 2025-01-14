package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Robot;
import frc.robot.Settings;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


public class Drivebase {
    Robot thisRobot;
    public SwerveDrive swerveDrive;
    public ChassisSpeeds drivebaseSpeeds;

    public Drivebase(Robot thisRobotIn) {
        double maximumSpeed = Units.feetToMeters(Settings.drivebaseMaxVelocityFPS);
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
        } catch (IOException e) {
            e.printStackTrace();
        }
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        thisRobot = thisRobotIn;
        drivebaseSpeeds = new ChassisSpeeds();
    }

    public void drive(double vx, double vy, double vr, boolean fieldCentric){
        drivebaseSpeeds.vxMetersPerSecond = vx;
        drivebaseSpeeds.vyMetersPerSecond = vy;
        drivebaseSpeeds.omegaRadiansPerSecond = vr;
        if(fieldCentric){
            thisRobot.drivebase.swerveDrive.driveFieldOriented(drivebaseSpeeds);
        }else{  
            thisRobot.drivebase.swerveDrive.drive(drivebaseSpeeds);
        }
    }


    public void initPath(String pathName){
        //create a path object from a path file

        //flip if you are on red

        //turn that path into a trajectory object

        //if we are a simulation
        //if we are a simulation set the robots pose to the starting pose of the path
        thisRobot.isSimulation();

        //update a variable to keep track of the fact we loaded a new path but have not begun to follow it
        boolean loadedPathHasStarted = false;

    }

    public boolean followLoadedPath(){
        //returns true if path is over

        //this is the first time we have followed the path, update loadedPathHasStarted and save the time the path started

        //get the elapsed time, currentTime - timePathStarted

        //sample the trajctory for the current goal state

        //get the goal chasisSpeeds from the trajectory and tell the robot to drive at that speed

        return false;
    }

}