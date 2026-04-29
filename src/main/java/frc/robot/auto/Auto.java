package frc.robot.auto;

import com.fasterxml.jackson.databind.JsonNode;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.auto.commands.FollowCurve;
import frc.robot.auto.commands.FullForceSwerve;
import frc.robot.auto.commands.RotateAndDriveTo;
import frc.robot.auto.commands.RotateToTag;
import frc.robot.auto.commands.Shoot;
import frc.robot.auto.commands.StartIntake;
import frc.robot.auto.routineManager.AutoRoutine;
import frc.robot.auto.routineManager.RoutineManager;
import frc.robot.utils.Vector2;
import frc.robot.utils.kade.LinearBezier;
import frc.robot.utils.trajectories.FeatherFlow;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.swerve.SwervePosition;

/**
 * Manages autonomous routines for the robot.
 * Uses {@link RoutineManager} to automatically detect and handle routines
 * annotated with {@link AutoRoutine}.
 */
public class Auto {
    public static RoutineManager routineManager;

    @AutoRoutine()
    public SequentialCommandGroup CluckRunRight() {
        Pigeon.setYaw(-90);

        return new SequentialCommandGroup(
            new StartIntake(),
            FeatherFlow.buildFeatherAuto("CluckRunRight",
                new Shoot(),
                new Shoot()
            )
        ); 
    }

    @AutoRoutine
    public SequentialCommandGroup backupAndShoot(){
        Pigeon.setYaw(0);

        return new SequentialCommandGroup(
            new RotateAndDriveTo(
                Pigeon.getRotationRad(), SwervePosition.getPosition().add(new Vector2(5*12, 0))
            ),
            new Shoot()
        );
    }   

    @AutoRoutine()
    public SequentialCommandGroup CluckRunLeft() {

        return new SequentialCommandGroup(
            FeatherFlow.buildFeatherAuto("CluckRunLeft", true, true,
                new Shoot(),
                new Shoot()
            )
        ); 
    }

    @AutoRoutine()
    public SequentialCommandGroup CluckScape() {

        return new SequentialCommandGroup(
            FeatherFlow.buildFeatherAuto("CluckScapeStart",
                new Shoot(),
                new Shoot()
            ),

            new SequentialCommandGroup(
                new FullForceSwerve(),
                new RotateToTag(.1, false)
            ),

            FeatherFlow.buildFeatherAuto("CluckScapeEnd", false, false, 
                new Shoot(),
                new Shoot()
            )
        ); 
    }

    
    @AutoRoutine()
    public SequentialCommandGroup StraightRight() {
        return Straight(true);
    }

    @AutoRoutine()
    public SequentialCommandGroup StraightLeft() {
        return Straight(false);
    }

    public SequentialCommandGroup Straight(boolean isRight) {
        Vector2 a  = parsePosition(new Vector2(500, 292));
        Vector2 b  = parsePosition(new Vector2(352, 292));
        Vector2 c  = parsePosition(new Vector2(352, 172));
        Vector2 d  = parsePosition(new Vector2(415, 292));
        Vector2 b2 = parsePosition(new Vector2(402, 292));
        Vector2 c2 = parsePosition(new Vector2(402, 172));

        double sign = isRight ? 1 : -1;
        a  = new Vector2(a.x,  sign * a.y);
        b  = new Vector2(b.x,  sign * b.y);
        c  = new Vector2(c.x,  sign * c.y);
        d  = new Vector2(d.x,  sign * d.y);
        b2 = new Vector2(b2.x, sign * b2.y);
        c2 = new Vector2(c2.x, sign * c2.y);

        Pigeon.setYaw(isRight ? 90 : -90 + 180);
        SwervePosition.setPosition(a);

        double intakeAngle = isRight ? 270 : -270 + 180;
        double pickupAngle = isRight ? -90  : 90 + 180;
        double returnAngle = isRight ? 0    : 180;

        return new SequentialCommandGroup(
            // --- Cycle 1 ---
            new FollowCurve(new LinearBezier(a, b),   returnAngle),
            new RotateAndDriveTo(intakeAngle),
            new StartIntake(),
            new FollowCurve(new LinearBezier(b, c),   pickupAngle),
            new FollowCurve(new LinearBezier(c, d),   returnAngle),
            new FollowCurve(new LinearBezier(d, a),   returnAngle),
            new Shoot(),

            // --- Cycle 2 ---
            new FollowCurve(new LinearBezier(a, b2),  returnAngle),
            new RotateAndDriveTo(intakeAngle),
            new StartIntake(),
            new FollowCurve(new LinearBezier(b2, c2), pickupAngle),
            new FollowCurve(new LinearBezier(c2, d),  returnAngle),
            new FollowCurve(new LinearBezier(d, a),   returnAngle),
            new Shoot()
        );
    }


    private static Vector2 parsePosition(Vector2 pos) {

        return new Vector2(
            pos.x - (Constants.FIELD_WIDTH/2),
            -((Constants.FIELD_HEIGHT/2) -pos.y)
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
