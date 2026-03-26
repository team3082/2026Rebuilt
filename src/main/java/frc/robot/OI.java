package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.controllermaps.LogitechF310;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterManager;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.ShooterManager.ShooterManagerState;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.subsystems.states.ShooterTarget;
import frc.robot.swerve.Odometry;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwervePID;
import frc.robot.utils.Vector2;

public class OI {
    private static Joystick driverStick;
    private static Joystick operatorStick;

    private static final int AXIS_DRIVE_X  = LogitechF310.AXIS_LEFT_X;
    private static final int AXIS_DRIVE_Y  = LogitechF310.AXIS_LEFT_Y;
    private static final int AXIS_ROTATE_X = LogitechF310.AXIS_RIGHT_X;
    private static final int BTN_SHOOT     = LogitechF310.BUTTON_RIGHT_BUMPER;
    private static final int BTN_ZERO_HOOD = LogitechF310.BUTTON_A;
    private static final int BTN_ZERO_GYRO = LogitechF310.BUTTON_Y;

    private static final int toggleIntake  = LogitechF310.BUTTON_LEFT_BUMPER;
    private static final int reverseIntake = LogitechF310.AXIS_LEFT_TRIGGER;
    private static final int intakeFeed    = LogitechF310.AXIS_RIGHT_TRIGGER;

    private static final int reverseIndexer = LogitechF310.BUTTON_X;
    private static final int defenseMode   = LogitechF310.BUTTON_RIGHT_BUMPER;

    private static final int manualTrenchToggle = LogitechF310.BUTTON_B;
    private static final int manualTowerToggle = LogitechF310.BUTTON_A;
    private static final int manualAimToggle = LogitechF310.BUTTON_Y;
    public static boolean manualAim = false;

    private static final double DRIVE_DEADBAND  = 0.05;
    private static final double ROTATE_DEADBAND = 0.05;
    private static final double ROTATE_SCALE     = 0.3;

    public static void init() {
        driverStick   = new Joystick(0);
        operatorStick = new Joystick(1);
    }

    public static void update() {
        handleDriverInput();
        operatorInput();
    }

    private static void handleDriverInput() {
        if (driverStick.getRawButton(BTN_ZERO_GYRO)) {
            Pigeon.reset();
        }

        if (driverStick.getRawButtonPressed(BTN_ZERO_HOOD)) {
            ShooterManager.zeroHood();
        }

        if (driverStick.getRawButton(BTN_SHOOT)) {
            ShooterManager.shoot();
            if (manualAim) {
                handleNormalDrive();
            } else {
                handleShootingMode();
            }
        } else {
            ShooterManager.stopShooting();
            handleNormalDrive();
        }

        if (driverStick.getRawButton(toggleIntake)) {
            Intake.startIntaking();
        } else if (driverStick.getRawAxis(reverseIntake) > 0.25) {
            Intake.reverse();
        } else if (driverStick.getRawAxis(intakeFeed) > 0.10) {
            Intake.startFeeding(driverStick.getRawAxis(intakeFeed));
        } else {
            if (Intake.getIntakeState() != IntakeState.IN_ROBOT) {
                Intake.stopIntaking();
            }
        }

        if (driverStick.getRawButton(reverseIndexer)) {
            Indexer.reverse();
        } else {
            Indexer.setNormalMode();
        }
    }

    private static void operatorInput() {
        if (operatorStick.getRawButtonPressed(defenseMode)) {
            if (Intake.getIntakeState() == IntakeState.IN_ROBOT) {
                Intake.stopIntaking();
            } else if (Intake.getIntakeState() == IntakeState.RESTING) {
                Intake.retract();
            }
        }

        if (operatorStick.getRawButton(manualTowerToggle)) {
            ShooterManager.setManualTower();
        } else if (operatorStick.getRawButton(manualTrenchToggle)) {
            ShooterManager.setManualTrench();
        } else {
            ShooterManager.setNormalAiming();
        }

        if (operatorStick.getRawButton(manualAimToggle)) {
            manualAim = true;
        } else {
            manualAim = false;
        }
    }

    private static void handleShootingMode() {
        Vector2 driveInput = getRawDriveVector();

        // Predict where the robot will be when the shot lands
        ShooterTarget target = ShooterManager.getTarget();
      
        Vector2 predictedPos = ShooterManager.predictRobotPos();
        double distance = target.pos.sub(predictedPos).mag();
        double timeOfFlight = distance * Tuning.Shooter.LOOK_AHEAD_TIME_K;

        Vector2 velocityCompensation = Odometry.getVelocity().mul(timeOfFlight);
        Vector2 shotTarget = target.pos.sub(velocityCompensation);
        Vector2 shotAim = shotTarget.sub(predictedPos);

        double targetAngle = Math.atan2(shotAim.y, shotAim.x) + Math.PI;

        SwervePID.setDestState(Odometry.getPosition(), targetAngle);

        if (driveInput.mag() > DRIVE_DEADBAND) {
            handleShootWhileMoving(driveInput);
        } else {
            handleShootWhileStationary();
        }
    }

    private static void handleShootWhileMoving(Vector2 driveInput) {
        SwerveManager.rotateAndDrive(SwervePID.updateOutputRot(), driveInput.mul(Constants.Swerve.shootWhileMoveSpeed));
        ShooterManager.shoot();
    }

    private static void handleShootWhileStationary() {
        if (SwervePID.atRot()) {
            SwerveManager.plant();
        } else {
            SwerveManager.rotateAndDrive(SwervePID.updateOutputRot(), new Vector2());
        }
        ShooterManager.shoot();
    }

    private static void handleNormalDrive() {
        Vector2 driveInput = getRawDriveVector();
        double  rotateInput = -driverStick.getRawAxis(AXIS_ROTATE_X) * ROTATE_SCALE;

        if (driveInput.mag() < DRIVE_DEADBAND)  driveInput  = new Vector2();
        if (Math.abs(rotateInput) < ROTATE_DEADBAND) rotateInput = 0;

        SwerveManager.rotateAndDrive(rotateInput, driveInput);
    }

    /**
     * Returns a drive vector from the left joystick with quadratic scaling applied
     * to each axis, preserving direction while reducing sensitivity near center.
     */
    private static Vector2 getRawDriveVector() {
        double x = driverStick.getRawAxis(AXIS_DRIVE_X);
        double y = driverStick.getRawAxis(AXIS_DRIVE_Y);
        return new Vector2(x * Math.abs(x), -y * Math.abs(y));
    }
}