package frc.robot.Subsystems;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Robot;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.util.Units;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


public class Drivebase {
    Robot thisRobot;
    public SwerveDrive swerveDrive;
    public Drivebase(Robot thisRobotIn) {
        double maximumSpeed = Units.feetToMeters(17.1);
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
        } catch (IOException e) {
            e.printStackTrace();
        }
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        thisRobot = thisRobotIn;
    }
   
}