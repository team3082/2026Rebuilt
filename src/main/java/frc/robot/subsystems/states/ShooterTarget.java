package frc.robot.subsystems.states;

import frc.robot.utils.Vector3;

public enum ShooterTarget {
    HUB(new Vector3(0, -145, 72)),
    PASS_LEFT(new Vector3(-230, -100, 0)),
    PASS_RIGHT(new Vector3(230, -100, 0));
    
    public final Vector3 pos;

    private ShooterTarget(Vector3 pos) {
        this.pos = pos;
    }
}
