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

    public static void update() {
        currentPattern.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }

    public enum Patterns{
        RED,
        GREEN,
        BLUE,
        ORANGE_FLASH
    }
    
    /**
     * Sets current pattern of the LEDs to a soild color, and applies to the LED strip
     */
    public static void setPattern(Patterns color){
        switch (color) {
            case RED:
                Color set_color = new Color(255, 0, 0);
                currentPattern = LEDPattern.solid(set_color);
                currentPattern.applyTo(m_ledBuffer);
                m_led.setData(m_ledBuffer);
                m_led.start();
                break;

            case GREEN:
                set_color = new Color(0, 0, 255);
                currentPattern = LEDPattern.solid(set_color);
                currentPattern.applyTo(m_ledBuffer);
                m_led.setData(m_ledBuffer);
                m_led.start();
                break;

            case BLUE:
                set_color = new Color(0, 255, 0);
                currentPattern = LEDPattern.solid(set_color);
                currentPattern.applyTo(m_ledBuffer);
                m_led.setData(m_ledBuffer);
                m_led.start();
                break;
            
            case ORANGE_FLASH:
                set_color = new Color(255, 0, 100);
                // creates a continuous gradient with two sections of orange on it, seperated by black, then makes it scroll
                currentPattern = LEDPattern.gradient(GradientType.kContinuous,
                    set_color, 
                    new Color(0,0,0), 
                    new Color(0,0,0), 
                    set_color, 
                    new Color(0,0,0), 
                    new Color(0,0,0)).scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(1),kLedSpacing);
                
                currentPattern.applyTo(m_ledBuffer);
                m_led.setData(m_ledBuffer);
                m_led.start();
                break;

            default:
                break;
        }
    }

}
