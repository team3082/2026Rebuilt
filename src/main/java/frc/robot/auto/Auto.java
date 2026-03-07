package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.commands.Shoot;
import frc.robot.auto.commands.StartIntake;
import frc.robot.auto.routineManager.AutoRoutine;
import frc.robot.auto.routineManager.RoutineManager;
import frc.robot.utils.trajectories.FeatherFlow;
import frc.robot.subsystems.sensors.Pigeon;

/**
 * Manages autonomous routines for the robot.
 * Uses {@link RoutineManager} to automatically detect and handle routines
 * annotated with {@link AutoRoutine}.
 */
public class Auto {
    public static RoutineManager routineManager;

    @AutoRoutine()
    public SequentialCommandGroup Shoot(){
        return new SequentialCommandGroup(
           new StartIntake(),
           new Shoot()
        );
    }

    @AutoRoutine()
    public SequentialCommandGroup Left3Piece() {
        Pigeon.setYaw(90);
        return new SequentialCommandGroup(
            new Shoot(),
            new StartIntake(),
            FeatherFlow.buildFeatherAuto("3 Piece Left",
                true, 
                new Shoot(),
                new Shoot(),
                new Shoot()
            )
        );
    }

    @AutoRoutine()
    public SequentialCommandGroup Right3Piece() {
        Pigeon.setYaw(-90);
        return new SequentialCommandGroup(
            new Shoot(),
            new StartIntake(),
            FeatherFlow.buildFeatherAuto("3 Piece Right", 
                new Shoot(),
                new Shoot(),
                new Shoot()
            )
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
    public SequentialCommandGroup MosesRight() {
        Pigeon.setYaw(-90);
        return new SequentialCommandGroup(
            // new StartIntake(),
            new Shoot(),
            FeatherFlow.buildFeatherAuto("Moses Right", 
                new Shoot(),
                new Shoot()
            )
        );
    }
    
    @AutoRoutine()
    public SequentialCommandGroup SilverLining(){
        Pigeon.setYaw(-90);
        return new SequentialCommandGroup(
            // new StartIntake(),
            new Shoot(),
            FeatherFlow.buildFeatherAuto("Silver Lining", 
                new Shoot(),
                new Shoot()
            )
        );

    }

    @AutoRoutine()
    public SequentialCommandGroup MosesLeft() {
        Pigeon.setYaw(90);
        return new SequentialCommandGroup(
            // new StartIntake(),
            new Shoot(),
            FeatherFlow.buildFeatherAuto("Moses Left", 
                true,
                new Shoot(),
                new Shoot()
            )
        );
    }

    @AutoRoutine()
    public SequentialCommandGroup Example() {
        Pigeon.setYaw(90);
        return new SequentialCommandGroup(
            new StartIntake(),
            FeatherFlow.buildFeatherAuto("Example", 
                true,
                new Shoot(),
                new Shoot()
            )
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
