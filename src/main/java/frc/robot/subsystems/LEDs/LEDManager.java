package frc.robot.subsystems.LEDs;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.lang.management.ManagementPermission;
import java.nio.MappedByteBuffer;
import java.util.Map;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.ShooterManager;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.states.ShooterState;

public class LEDManager {
    public static AddressableLED m_led;
    public static AddressableLEDBuffer m_ledBuffer;
    public static LEDPattern currentPattern;
    public static Distance kLedSpacing = Distance.ofBaseUnits(1.0 / 47.0, Meters);

   /*public static void setSolid(Color8Bit color){
        Color set_color = new Color(color.red, color.blue, color.green);
        currentPattern = LEDPattern.solid(set_color);
    }
/* */
    public static void init() {
        m_led = new AddressableLED(0);
        m_ledBuffer = new AddressableLEDBuffer(47);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.start();
        setColor(Colors.BLUE);
        GameState();

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
        YELLOW,
        CYAN,
        MAGENTA,
        WHITE,
        RED_CYAN,
        GREEN_CYAN,
        BLUE_CYAN,
        YELLOW_CYAN,
        MAGENTA_CYAN,
        WHITE_CYAN,
        //animations
        RED_SCROLL,
        YELLOW_SCROLL,
        RED_CYAN_SCROLL,
        YELLOW_CYAN_SCROLL,
        RAINBOW
    }
    
    public static void setColor(Colors color){
        switch (color) {
            case RED:
                Color set_color = new Color(255, 0, 0);
                currentPattern = LEDPattern.solid(set_color);
                
                
                break;

            case GREEN:
                set_color = new Color(0, 255, 0);
                currentPattern = LEDPattern.solid(set_color);
                
                break;

            case BLUE:
                set_color = new Color(0, 0, 255);
                currentPattern = LEDPattern.solid(set_color);
                
                
                break;

            case YELLOW:
                set_color = new Color(255, 255, 0);
                currentPattern = LEDPattern.solid(set_color);

                break;
            
            case CYAN:
                set_color = new Color(0, 255, 255);
                currentPattern = LEDPattern.solid(set_color);
                
                
                break;
            case MAGENTA:
                set_color = new Color(255, 0, 255);
                currentPattern = LEDPattern.solid(set_color);
                
                
                break;
            case WHITE:
                set_color = new Color(255, 255, 255);
                currentPattern = LEDPattern.solid(set_color);
                
                
                break;
            case RED_CYAN:
                Color set_color1 = new Color(255, 0, 0);
                Color set_color2 = new Color(0, 255, 255);
                currentPattern = LEDPattern.steps(Map.of(0, set_color1, 0.125, set_color2, 0.25, set_color1, 0.375, set_color2, 0.5, set_color1, 0.625, set_color2, 0.75, set_color1, 0.875, set_color2)).scrollAtAbsoluteSpeed(LinearVelocity.ofBaseUnits(0.15,MetersPerSecond),kLedSpacing);
                
                break;
            case GREEN_CYAN:
                set_color1 = new Color(0, 255, 0);
                set_color2 = new Color(0, 255, 255);
                currentPattern = LEDPattern.steps(Map.of(0, set_color1, 0.125, set_color2, 0.25, set_color1, 0.375, set_color2, 0.5, set_color1, 0.625, set_color2, 0.75, set_color1, 0.875, set_color2)).scrollAtAbsoluteSpeed(LinearVelocity.ofBaseUnits(0.15,MetersPerSecond),kLedSpacing);
                
                break;
            case BLUE_CYAN:
                set_color1 = new Color(0, 0, 255);
                set_color2 = new Color(0, 255, 255);
                currentPattern = LEDPattern.steps(Map.of(0, set_color1, 0.125, set_color2, 0.25, set_color1, 0.375, set_color2, 0.5, set_color1, 0.625, set_color2, 0.75, set_color1, 0.875, set_color2)).scrollAtAbsoluteSpeed(LinearVelocity.ofBaseUnits(0.15,MetersPerSecond),kLedSpacing);
                
                break;
            case YELLOW_CYAN:
                set_color1 = new Color(255, 255, 0);
                set_color2 = new Color(0, 255, 255);
                currentPattern = LEDPattern.steps(Map.of(0, set_color1, 0.125, set_color2, 0.25, set_color1, 0.375, set_color2, 0.5, set_color1, 0.625, set_color2, 0.75, set_color1, 0.875, set_color2)).scrollAtAbsoluteSpeed(LinearVelocity.ofBaseUnits(0.15,MetersPerSecond),kLedSpacing);
                break;
            case MAGENTA_CYAN:
                set_color1 = new Color(255, 0, 255);
                set_color2 = new Color(0, 255, 255);
                currentPattern = LEDPattern.steps(Map.of(0, set_color1, 0.125, set_color2, 0.25, set_color1, 0.375, set_color2, 0.5, set_color1, 0.625, set_color2, 0.75, set_color1, 0.875, set_color2)).scrollAtAbsoluteSpeed(LinearVelocity.ofBaseUnits(0.15,MetersPerSecond),kLedSpacing);

                break;
            case WHITE_CYAN:
                set_color1 = new Color(255, 255, 255);
                set_color2 = new Color(0, 255, 255);
                currentPattern = LEDPattern.steps(Map.of(0, set_color1, 0.125, set_color2, 0.25, set_color1, 0.375, set_color2, 0.5, set_color1, 0.625, set_color2, 0.75, set_color1, 0.875, set_color2)).scrollAtAbsoluteSpeed(LinearVelocity.ofBaseUnits(0.15,MetersPerSecond),kLedSpacing);
                
                break;
            case RED_SCROLL:
                set_color1 = new Color(255, 0, 0);
                set_color2 = new Color(0, 0, 0);
                currentPattern = LEDPattern.gradient(GradientType.kContinuous, set_color1, set_color2, set_color1, set_color2, set_color1, set_color2).scrollAtAbsoluteSpeed(LinearVelocity.ofBaseUnits(0.15,MetersPerSecond),kLedSpacing);
                
                break;
            case RAINBOW:
                currentPattern = LEDPattern.rainbow(255, 200);
                
            default:
                break;
        }
    }

    public enum GameState{
        PREGAME,
        AUTO,
        TELEOP
    }

    public static GameState currGameState;

    public static void GameState(){
        if (DriverStation.isAutonomous()){
            currGameState = GameState.AUTO;
        }
        if (DriverStation.isTeleop()){
            currGameState = GameState.TELEOP;
        }
        if (DriverStation.isTest()){
            currGameState = GameState.PREGAME;
        }
    }
    public static void update(){
        GameState();
        currentPattern.applyTo(m_ledBuffer);
        for (int i = 0; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i, m_ledBuffer.getRed(i), m_ledBuffer.getBlue(i), m_ledBuffer.getGreen(i));
        }
        m_led.setData(m_ledBuffer);
    }
 
}

