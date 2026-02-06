package frc.robot.subsystems.visualizer;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class WheelMech {
    private double currentAngle;
    private final int numSpokes;
    private final int numRimSegments;
    private final MechanismLigament2d[] spokes;
    private final MechanismLigament2d[] rimRadials;
    private final MechanismLigament2d[] rimSegments;
    private final double spokeAngleSpacing;
    private final double rimAngleSpacing;
    private final double width;
    private final double radius;

    public WheelMech(String name, MechanismObject2d base, int numSpokes, double radius, double width, Color8Bit color) {
        this.numSpokes = numSpokes;
        this.radius = radius;
        this.spokes = new MechanismLigament2d[numSpokes];
        this.spokeAngleSpacing = 360.0 / numSpokes;
        this.width = width;

        this.numRimSegments = 12;
        this.rimRadials = new MechanismLigament2d[numRimSegments];
        this.rimSegments = new MechanismLigament2d[numRimSegments];
        this.rimAngleSpacing = 360.0 / numRimSegments;

        double chordLength = 2 * radius * Math.sin(Math.toRadians(rimAngleSpacing / 2.0));
        double edgeAngle = (180.0 - rimAngleSpacing) / 2.0;

        for (int index = 0; index < numSpokes; index++) {
            double startAngle = index * spokeAngleSpacing;
            
            spokes[index] = base.append(new MechanismLigament2d(name + "_spoke" + index, radius, startAngle));
            spokes[index].setColor(color);
            spokes[index].setLineWeight(width);
        }

        for (int index = 0; index < numRimSegments; index++) {
            double rimStartAngle = index * rimAngleSpacing;
            
            rimRadials[index] = base.append(
                new MechanismLigament2d(name + "_rimRadial" + index, radius, rimStartAngle)
            );
            rimRadials[index].setColor(new Color8Bit(0, 0, 0)); 
            rimRadials[index].setLineWeight(0);
            
            rimSegments[index] = rimRadials[index].append(
                new MechanismLigament2d(name + "_rim" + index, chordLength, 180 - edgeAngle)
            );
            rimSegments[index].setColor(color);
            rimSegments[index].setLineWeight(width * 1.5);
        }
    }

    public void update(double velocity) {
        currentAngle += velocity;
        currentAngle %= 360;

        for (int index = 0; index < numSpokes; index++) {
            double offsetAngle = (index * spokeAngleSpacing) + currentAngle;
            spokes[index].setAngle(offsetAngle);
        }

        for (int index = 0; index < numRimSegments; index++) {
            double offsetAngle = (index * rimAngleSpacing) + currentAngle;
            rimRadials[index].setAngle(offsetAngle);
        }
    }
}