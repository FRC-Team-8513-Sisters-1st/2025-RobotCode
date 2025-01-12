package frc.robot.Logic;

import frc.robot.Settings;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class TeleopController {

    Robot thisRobot;
    Joystick driverXboxController = new Joystick(Settings.driverJoystickPort);

    public TeleopController(Robot thisRobotIn){
        thisRobot = thisRobotIn;
    }

    public void driveTele() {
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
        SmartDashboard.putNumber("xv", xV);
        SmartDashboard.putNumber("yv", yV);
        SmartDashboard.putNumber("rv", rV);
        thisRobot.drivebase.betaDrive(xV,yV,rV);
    }
}
