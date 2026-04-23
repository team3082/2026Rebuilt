package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

// AUTO
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterManager;
import frc.robot.auto.Auto;
    
// SUBSYSTEMS
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwervePID;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.RTime;
import frc.robot.utils.Vector2;
import frc.robot.utils.trajectories.FeatherFlow;
import frc.robot.vision.VisionManager;

public class Robot extends TimedRobot {
  @SuppressWarnings("resource")
  public Robot() {
    if (Robot.isReal()){
      try {
        Thread.sleep(5000);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
    RTime.init();
    Auto.init();
    FeatherFlow.init();

    // Swerve
    Pigeon.init();
    SwerveManager.init();
    SwervePosition.init();
    SwervePID.init();
    SwervePosition.setPosition(new Vector2());

    // Subsystems
    ShooterManager.init();
    Indexer.init();
    Intake.init();

    // Logging
    Telemetry.init();

    //LEDs
    // LEDManager.init();
    // LEDMech2D.init();
    
    // Controls
    OI.init();
  }

  @Override
  public void robotPeriodic() {
    // LEDMech2D.update();
    RTime.update();
    Pigeon.update();
    ShooterManager.update();
    Indexer.update();
    Intake.update();
    Telemetry.update();
    VisionManager.poll();
    SwervePosition.update();
    SwerveManager.update();
    // LEDManager.update();
  }

  @Override
  public void autonomousInit() {
    Auto.startRoutine();
  }
  
  @Override
  public void autonomousPeriodic() {
    Auto.update();
  }

  @Override
  public void teleopInit() {
    // Intake.stopIntaking();
  }

  @Override
  public void teleopPeriodic() {
    OI.update();

  }

  @Override
  public void disabledInit() {
    // Clear Auto Commands
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().disable();

    // Disable Subsystems
    SwerveManager.rotateAndDrive(0, new Vector2());
    if (Robot.isSimulation()) {
      SwerveManager.mods[0].simModule.speed = 0;
      SwerveManager.mods[1].simModule.speed = 0;
      SwerveManager.mods[2].simModule.speed = 0;
      SwerveManager.mods[3].simModule.speed = 0;
    }

  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

}
