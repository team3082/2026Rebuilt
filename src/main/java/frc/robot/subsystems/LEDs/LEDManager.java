package frc.robot.subsystems.LEDs;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LEDManager {
    public static AddressableLED m_led;
    public static AddressableLEDBuffer m_ledBuffer;
    public static LEDPattern currentPattern;
    public static Distance kLedSpacing = Distance.ofBaseUnits(1.0 / 50.0, Meters);

   /*public static void setSolid(Color8Bit color){
        Color set_color = new Color(color.red, color.blue, color.green);
        currentPattern = LEDPattern.solid(set_color);
    }
/* */
    public static void init() {
        m_led = new AddressableLED(0);
        m_ledBuffer = new AddressableLEDBuffer(47);
        m_led.setLength(m_ledBuffer.getLength());
    }

        // kGreen = blue, kBue = green
        
   /* // Set the data
        currentPattern.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public static void update() {
        currentPattern.applyTo(m_ledBuffer);
        // Set the LEDs
        m_led.setData(m_ledBuffer);
        
    }
*/
   

    public enum Colors{
        RED,
        GREEN,
        BLUE,
        YELLOW
    }
    
    public static void setColor(Colors color){
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

            case YELLOW:
                set_color = new Color(255, 0, 255);
                currentPattern = LEDPattern.solid(set_color);
                currentPattern.applyTo(m_ledBuffer);
                m_led.setData(m_ledBuffer);
                m_led.start();
                LEDMech2D.mechLED(new Color8Bit(255, 255, 0));
                break;

            default:
                break;
        }
    }

}

