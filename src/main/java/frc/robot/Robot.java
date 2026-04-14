package frc.robot;

import java.io.IOException;
import java.nio.file.AtomicMoveNotSupportedException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.utils.auto.CommandLoader;
import frc.robot.utils.auto.CommandLoader.CommandConstructorInfo;
import frc.robot.utils.trajectories.FeatherFlow;

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

    ArrayList<CommandConstructorInfo> info = CommandLoader.loadCommandConstructors();
    info.forEach(System.out::println);
    SmartDashboard.putData(new Field2d());

    try {
      ObjectMapper mapper = new ObjectMapper();
      Path target = Path.of("src/main/deploy/command_constructors.json");
      Path tmp = Files.createTempFile(target.getParent() == null ? Path.of(".") : target.getParent(),
                                      "command_constructors", ".tmp");
      // write to temp file first
      mapper.writerWithDefaultPrettyPrinter().writeValue(tmp.toFile(), info);
      // then move into place (atomic if supported)
      try {
        Files.move(tmp, target, StandardCopyOption.REPLACE_EXISTING, StandardCopyOption.ATOMIC_MOVE);
      } catch (AtomicMoveNotSupportedException ex) {
        Files.move(tmp, target, StandardCopyOption.REPLACE_EXISTING);
      }
      System.out.println("Wrote JSON to " + target.toAbsolutePath());
    } catch (IOException e) {
      e.printStackTrace();
    }
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
