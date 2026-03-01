package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.auto.commands.Shoot;
import frc.robot.auto.commands.StartIntake;
import frc.robot.auto.routineManager.AutoRoutine;
import frc.robot.auto.routineManager.RoutineManager;
import frc.robot.utils.Vector2;
import frc.robot.utils.trajectories.FeatherFlow;
import frc.robot.subsystems.Intake;

/**
 * Manages autonomous routines for the robot.
 * Uses {@link RoutineManager} to automatically detect and handle routines
 * annotated with {@link AutoRoutine}.
 */
public class Auto {
    public static RoutineManager routineManager;

    @AutoRoutine()
    public SequentialCommandGroup AutoSweep(){
        System.out.println("Starting Test Path Auto Routine");

        return new SequentialCommandGroup(
            FeatherFlow.buildFeatherAuto(
                "Auto Sweep", 
                new InstantCommand(() -> {
                    Intake.startIntaking();
                }),
                new InstantCommand(() -> {
                    Intake.stopIntaking();
                }),
                new Shoot(),
                new InstantCommand(() -> {
                    Intake.startIntaking();
                }),
                new InstantCommand(() -> {
                    Intake.stopIntaking();
                }),
                new Shoot()
            )
        );
    }

     @AutoRoutine()
    public SequentialCommandGroup Shoot(){
        return new SequentialCommandGroup(
           new StartIntake(),
           new Shoot()
        );
    }

    @AutoRoutine()
    public SequentialCommandGroup Left3Piece() {
        
        return new SequentialCommandGroup(
            new StartIntake(),
            FeatherFlow.buildFeatherAuto("3 Piece Left (Copy)", 
            
            new Shoot(),
            new Shoot(),
            new Shoot())
        );
    }

    @AutoRoutine()
    public SequentialCommandGroup Right3Piece() {
        
        return new SequentialCommandGroup(
            new StartIntake(),
            FeatherFlow.buildFeatherAuto("3 Piece Right", 
            
            new Shoot(),
            new Shoot(),
            new Shoot())
        );
    }

     @AutoRoutine()
    public SequentialCommandGroup Stupid() {
        
        return new SequentialCommandGroup(
            new StartIntake(),
            FeatherFlow.buildFeatherAuto("Untitled", 
            
            new Shoot(),
            new Shoot(),
            new Shoot())
        );
    }

    @AutoRoutine()
    public SequentialCommandGroup DepotToCenter() {
        return new SequentialCommandGroup(
            new StartIntake(),
            FeatherFlow.buildFeatherAuto("Depot to Center", 

            new Shoot(),
            new Shoot(),
            new Shoot()
            ));
    }

    @AutoRoutine()
    public SequentialCommandGroup CenterToDepot() {
        return new SequentialCommandGroup(
            new StartIntake(),
            FeatherFlow.buildFeatherAuto("Center to Depot", 

            new Shoot(),
            new Shoot()
            ));
    }

    @AutoRoutine()
    public SequentialCommandGroup Test1() {
        return new SequentialCommandGroup(
            FeatherFlow.buildFeatherAuto("Untitled"

            ));
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
