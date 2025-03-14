package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;
import frc.robot.Logic.Enums.LEDColors;

public class LEDs {

    Robot thisRobot;
    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;

    public LEDColors color = LEDColors.purple;


    public LEDs(Robot thisRobotIn) {
        thisRobot = thisRobotIn;

        m_led = new AddressableLED(9);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(255);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();

    }

    public void updateLEDColor(LEDColors ledColor) {
        switch (ledColor) {
            case green:
                LEDPattern green = LEDPattern.solid(Color.kLimeGreen);
                // Apply the LED pattern to the data buffer
                green.applyTo(m_ledBuffer);
                break;
            case red:
                LEDPattern red = LEDPattern.solid(Color.kCrimson);
                // Apply the LED pattern to the data buffer
                red.applyTo(m_ledBuffer);
                break;
            case pink:
                LEDPattern pink = LEDPattern.solid(Color.kLightPink);
                // Apply the LED pattern to the data buffer
                pink.applyTo(m_ledBuffer);
                break;
            case blue:
                LEDPattern blue = LEDPattern.solid(Color.kCornflowerBlue);
                // Apply the LED pattern to the data buffer
                blue.applyTo(m_ledBuffer);
                break;
            case purple:
                LEDPattern purple = LEDPattern.solid(Color.kLavender);
                // Apply the LED pattern to the data buffer
                purple.applyTo(m_ledBuffer);
                break;
            default:
                break;
        }
        m_led.setData(m_ledBuffer);
    }
}

