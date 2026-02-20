package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.Tuning;

public class Indexer {
    private static TalonFX spindexerMotor;
    private static TalonFX handoffMotor;

    public static void init() {
        spindexerMotor = new TalonFX(Constants.Indexer.SPINDEXER_ID);
        spindexerMotor.getConfigurator().apply(new TalonFXConfiguration());
        TalonFXConfiguration spindexerConfiguration = new TalonFXConfiguration();
        spindexerConfiguration.CurrentLimits.StatorCurrentLimit = 120;
        spindexerConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        spindexerMotor.getConfigurator().apply(spindexerConfiguration);

        handoffMotor = new TalonFX(Constants.Indexer.HANDOFF_ID);
        handoffMotor.getConfigurator().apply(new TalonFXConfiguration());
        TalonFXConfiguration handoffConfiguration = new TalonFXConfiguration();
        spindexerConfiguration.CurrentLimits.StatorCurrentLimit = 100;
        spindexerConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        handoffMotor.getConfigurator().apply(handoffConfiguration);

    }

    public static void update() {
        switch (ShooterManager.getShooterState()) {
            case SHOOTING:
                spindexerMotor.set(Tuning.Indexer.SPINDEXER_SPEED);
                handoffMotor.set(Tuning.Indexer.HANDOFF_SPEED);
                break;

            default:
                spindexerMotor.set(0);
                handoffMotor.set(0);
                break;
        }
    }

    public static double getSpindexerSpeed() {
        return spindexerMotor.get();
    }

    public static double getHandoffSpeed() {
        return handoffMotor.get();
    }
}