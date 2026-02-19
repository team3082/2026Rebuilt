package frc.robot.vision;

import frc.robot.Constants;

//import frc.robot.Constants.REEF_POSITIONS;
import frc.robot.utils.Vector2;
import frc.robot.utils.Vector3;

public class AprilTag {

    private Vector3 pos;
    private double rot;
    private int id;

    /**
     * Represents an Apriltag on the field
     * @param x
     * @param y
     * @param z
     * @param rot Angle around vertical axis
     */
    public AprilTag(int id, double x, double y, double z, double rot) {

        this.id = id;
        this.pos = new Vector3(x - (Constants.FIELD_HEIGHT / 2), y - (Constants.FIELD_WIDTH / 2), z);
        this.rot = rot;
    }

    /**
     * Returns the x and y field coordinates of the Apriltag
     * @return Vector2(x, y)
     */
    public Vector2 getPosition(){
        return new Vector2(pos.x, pos.y);
    }

    /**
     * returns the id of the apriltag
     * @return the id of this tag
     */
    public int getID() {
        return id;
    }
    
    /**
     * Returns the x, y and z field coordinates of the Apriltag
     * @return Vector3(x, y, z)
     */
    public Vector3 get3DPosition(){
        return pos;
    }

    /**
     * Returns the angle of the Apriltag around the vertical axis 
     * @return rot
     */
    public double getRotation(){
        return rot;
    }

}
