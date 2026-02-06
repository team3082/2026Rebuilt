package frc.robot.subsystems.visualizer;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class WheelMech {
    private double currentAngle;
    private final int numSpokes;
    private final MechanismLigament2d[] spokes;
    private final double spokeAngleSpacing;
    private final double width;

    public WheelMech(String name, MechanismObject2d base, int numSpokes, double length, double width, Color8Bit color) {
        this.numSpokes = numSpokes;
        this.spokes = new MechanismLigament2d[numSpokes];
        this.spokeAngleSpacing = 360.0 / numSpokes;
        this.width = width;

        double edgeAngle = (180.0 - spokeAngleSpacing) / 2.0;

        double chordLength = 2 * length * Math.sin(Math.toRadians(spokeAngleSpacing / 2.0));

        for (int index = 0; index < numSpokes; index++) {
            double startAngle = index * spokeAngleSpacing;
            
            spokes[index] = base.append(new MechanismLigament2d(name + index, length, startAngle));
            spokes[index].setColor(color);
            spokes[index].setLineWeight(width);

            MechanismLigament2d rimSegment = spokes[index].append(
                new MechanismLigament2d(name + index + "rim", chordLength, 180 - edgeAngle)
            );
            rimSegment.setColor(color);
            rimSegment.setLineWeight(width);
        }
    }

    /**
     * Updates the wheel rotation.
     * @param velocity The speed of the wheel (e.g., in degrees per second or a scaled factor)
     */
    public void update(double velocity) {
        currentAngle += velocity;
        currentAngle %= 360;

        for (int index = 0; index < numSpokes; index++) {
            double offsetAngle = (index * spokeAngleSpacing) + currentAngle;
            spokes[index].setAngle(offsetAngle);
        }
    }
}