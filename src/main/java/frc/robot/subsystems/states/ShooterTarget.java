package frc.robot.subsystems.states;

import frc.robot.utils.Vector2;

public enum ShooterTarget {
    HUB(new Vector2(143.5, 0)),
    PASS_LEFT(new Vector2(230, -100)),
    PASS_RIGHT(new Vector2(230, 100));
    
    public final Vector2 pos;

    private ShooterTarget(Vector2 pos) {
        this.pos = pos;
    }
}
