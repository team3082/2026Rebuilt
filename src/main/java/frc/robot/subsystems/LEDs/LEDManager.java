package frc.robot.subsystems.LEDs;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LEDManager {
    public static AddressableLED m_led;
    public static AddressableLEDBuffer m_ledBuffer;
    public static LEDPattern currentPattern;
    public static Distance kLedSpacing = Units.Meters.of(1 / 50.0);


    public static void init() {
        m_led = new AddressableLED(0);
        m_ledBuffer = new AddressableLEDBuffer(47);
        m_led.setLength(m_ledBuffer.getLength());
    }

    public enum Patterns{
        RED,
        GREEN,
        BLUE
    }
    
    /**
     * Sets current pattern of the LEDs to a soild color, applies to the LED strip and the Mech2D
     */
    public static void setColor(Patterns color){
        switch (color) {
            case RED:
                Color set_color = new Color(255, 0, 0);
                currentPattern = LEDPattern.solid(set_color);
                currentPattern.applyTo(m_ledBuffer);
                m_led.setData(m_ledBuffer);
                m_led.start();
                LEDMech2D.mechLED(new Color8Bit(255, 0, 0));
                break;

            case GREEN:
                set_color = new Color(0, 0, 255);
                currentPattern = LEDPattern.solid(set_color);
                currentPattern.applyTo(m_ledBuffer);
                m_led.setData(m_ledBuffer);
                m_led.start();
                LEDMech2D.mechLED(new Color8Bit(0, 255, 0));
                break;

            case BLUE:
                set_color = new Color(0, 255, 0);
                currentPattern = LEDPattern.solid(set_color);
                currentPattern.applyTo(m_ledBuffer);
                m_led.setData(m_ledBuffer);
                m_led.start();
                LEDMech2D.mechLED(new Color8Bit(0, 0, 255));
                break;

            default:
                break;
        }
    }

}
