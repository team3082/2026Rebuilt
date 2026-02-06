package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

// AUTO
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterManager;
import frc.robot.auto.Auto;
// SUBSYSTEMS
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.subsystems.states.ShooterState;
import frc.robot.subsystems.states.ShooterTarget;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwervePID;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.RTime;
import frc.robot.utils.Vector2;

public class Robot extends LoggedRobot {
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
    
    
    ShooterManager.shooterState = ShooterState.REVVING;
    ShooterManager.target = ShooterTarget.PASS_RIGHT;
    
    // Controls
    OI.init();




    Logger.recordMetadata("ProjectName", "2026Rebuilt"); // Set a metadata value
    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else if (Constants.REPLAY) {
      setUseTiming(true);
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    } else {
      Logger.addDataReceiver(new NT4Publisher());
    }

    Logger.start(); // Start logging

  }

  @Override
  public void robotPeriodic() {
    
    RTime.update();
    Pigeon.update();
    SwervePosition.update();
    SwerveManager.update();
    ShooterManager.update();
    Indexer.update();
    Intake.update();
    Telemetry.update();

  

  }

  @Override
  public void autonomousInit() {
    Auto.startRoutine();
    Intake.startIntaking();
  }
  
  @Override
  public void autonomousPeriodic() {
    // Auto.update();
    SwerveManager.rotateAndDrive(0, new Vector2(1, 1));
  }

  @Override
  public void teleopInit() {
    Intake.stopIntaking();
  }

  @Override
  public void teleopPeriodic() {
    OI.userInput();

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
