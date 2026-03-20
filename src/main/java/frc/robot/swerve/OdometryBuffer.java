package frc.robot.swerve;

import frc.robot.utils.Vector2;

public class OdometryBuffer {
    private final int BUFFER_SIZE = 2; 
    private Vector2[] buffer = new Vector2[BUFFER_SIZE];

    private final int VEL_BUFFER_SIZE = 5; 
    private Vector2[] velBuffer = new Vector2[VEL_BUFFER_SIZE];

    public OdometryBuffer() {
        for (int i = 0; i < VEL_BUFFER_SIZE; i++) {
            velBuffer[i] = new Vector2();
        }
    }

    public void addValue(Vector2 newValue) {
        Vector2[] initialBuffer = this.buffer.clone();
        buffer[0] = newValue;
        for (int i = 1; i < BUFFER_SIZE; i++) {
            buffer[i] = initialBuffer[i - 1];
        }

        Vector2[] initialVelBuffer = this.velBuffer.clone();
        velBuffer[0] = newValue;
        for (int i = 1; i < VEL_BUFFER_SIZE; i++) {
            velBuffer[i] = initialVelBuffer[i - 1];
        }
    }

    public Vector2 getTotalBuffer() {
        Vector2 totalBuffer = new Vector2();
        for (Vector2 vector : buffer) {
            totalBuffer = totalBuffer.add(vector);
        }
        return totalBuffer;
    }

    public Vector2 getVelocity() {
        Vector2 total = new Vector2();
        for (Vector2 v : velBuffer) {
            total = total.add(v);
        }

        return total.div(VEL_BUFFER_SIZE);
    }
}