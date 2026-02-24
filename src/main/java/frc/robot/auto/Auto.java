package frc.robot.auto;

import java.util.List;
import java.util.Vector;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.auto.commands.FollowPath;
import frc.robot.auto.routineManager.AutoRoutine;
import frc.robot.auto.routineManager.RoutineManager;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.Vector2;
import frc.robot.utils.trajectories.CubicBezierCurve;
import frc.robot.utils.trajectories.FeatherEvent;
import frc.robot.utils.trajectories.FeatherFlow;
import frc.robot.utils.trajectories.RobotPath;
/**
 * Manages autonomous routines for the robot.
 * Uses {@link RoutineManager} to automatically detect and handle routines
 * annotated with {@link AutoRoutine}.
 */
public class Auto {
    public static RoutineManager routineManager;

    @AutoRoutine()
    public  SequentialCommandGroup basicAuto(){
        System.out.println("Starting Test Path Auto Routine");

        return new SequentialCommandGroup(
            FeatherFlow.buildFeatherAuto(
                "BasicAuto", 
                new InstantCommand(() -> {
                    System.out.println("Debug One");
                }),
                new InstantCommand(() -> {
                    System.out.println("Debug Two");
                })
            ),
            new WaitCommand(5)
        );
    }

    @AutoRoutine
    public SequentialCommandGroup Smally(){
        System.out.println("Starting Long Line");

        return FeatherFlow.buildFeatherAuto(
            "Smally", 
            new InstantCommand(() -> {
                System.out.println("Debug One");
            }),
            new InstantCommand(() -> {
                System.out.println("Debug Two");
            })
        );
    }

    @AutoRoutine()
    public  SequentialCommandGroup exampleAuto(){
        System.out.println("Starting Test Path Auto Routine");

        return FeatherFlow.buildFeatherAuto(
            "ExampleAuto", 
            new InstantCommand(() -> {
                System.out.println("Debug One");
            }),
            new InstantCommand(() -> {
                System.out.println("Debug Two");
            })
        );
    }

    @AutoRoutine()
    public  SequentialCommandGroup testTwo(){
        System.out.println("Starting Test Two Auto Routine");

        return FeatherFlow.buildFeatherAuto(
            "Test2", 
            new InstantCommand(() -> {
                System.out.println("Debug One");
            }),
            new InstantCommand(() -> {
                System.out.println("Debug Two");
            }),
            new InstantCommand(() -> {
                System.out.println("Debug Three");
            }),
            new InstantCommand(() -> {
                System.out.println("Debug Four");
            })
        );
    }

    @AutoRoutine()
    public  SequentialCommandGroup untitled3(){
        System.out.println("Starting Test Two Auto Routine");

        return FeatherFlow.buildFeatherAuto(
            "Untitled 3", 
            new InstantCommand(() -> {
                System.out.println("Debug One");
            }),
            new InstantCommand(() -> {
                System.out.println("Debug Two");
            }),
            new InstantCommand(() -> {
                System.out.println("Debug Three");
            }),
            new InstantCommand(() -> {
                System.out.println("Debug Four");
            })
        );
    }

    @AutoRoutine()
    public SequentialCommandGroup Silly(){
        System.out.println("Starting Test Two Auto Routine");

        return FeatherFlow.buildFeatherAuto(
            "Silly", 
            new InstantCommand(() -> {
                System.out.println("Debug One");
            }),
            new InstantCommand(() -> {
                System.out.println("Debug Two");
            }),
            new InstantCommand(() -> {
                System.out.println("Debug Three");
            }),
            new InstantCommand(() -> {
                System.out.println("Debug Four");
            })
        );
    }


    private static Vector2 parseFieldPosition(Vector2 posNode) {
        return new Vector2(
            posNode.x - (Constants.FIELD_WIDTH/2),
            -((Constants.FIELD_HEIGHT/2) - posNode.y)
        );
    }

    /**
     * Gets the auto selector from {@link RoutineManager}
     * @return SendableChooser<String>
     */
    public static SendableChooser<String> getAutoSelector(){
        return routineManager.getAutoSelector();
    }

    /**
     * Initializes the autonomous system by creating a {@link RoutineManager}
     * instance and registering all routines in this class.
     */
    public static void init() {
        routineManager = new RoutineManager();
        routineManager.addClass(new Auto());
    }

    /**
     * Schedules the currently selected autonomous command.
     * Should be called at the start of autonomous mode.
     */
    public static void startRoutine() {
        CommandScheduler.getInstance().enable();
        routineManager.getCurrentCommand().schedule();
    }

    /**
     * Runs the {@link CommandScheduler}, ensuring commands are executed.
     * Should be called periodically during autonomous mode.
     */
    public static void update() {
        CommandScheduler.getInstance().run();
    }

    /**
     * Disables the {@link CommandScheduler}, ensuring commands are turned off.
     * Should be called during robot disabled
     */
    public static void disable() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().disable();
    }
}
