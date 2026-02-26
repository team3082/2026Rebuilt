package frc.robot.utils.trajectories;

import java.util.ArrayList;
import java.util.List;

import frc.robot.utils.Vector2;

/**
 * Represents a path for a robot to follow, defined by a sequence of points.
 * Provides interpolation, subpath extraction, and splitting functionality.
 */
public class RobotPath {
    private List<Vector2> points;
    private List<Double> curvatures;
    private List<Double> lengths;
    private List<Double> rotations;
    
    private double totalLength;
    
    /**
     * Creates a new RobotPath from a list of points.
     * 
     * @param points List of Vector2 points defining the path
     */
    public RobotPath(List<Vector2> points, List<Double> curvatures) {
        this.points = new ArrayList<>(points);
        this.curvatures = new ArrayList<>(curvatures);

        this.totalLength = calculateTotalLength();
    }

    /**
     * Calculates the total length of the path as the sum of distances between consecutive points.
     * @return Total length of the path
     */
    private double calculateTotalLength() {
        double length = 0.0;
        lengths = new ArrayList<>();
        lengths.add(length);

        for (int i = 1; i < points.size(); i++) {
            length += points.get(i).dist(points.get(i-1));
            lengths.add(length);
        }

        return length;
    }
    
    /**
     * Gets a point along the path corresponding to a parameter t in [0, 1].
     * Performs linear interpolation between adjacent points.
     * 
     * @param t Fraction of the path (0 = start, 1 = end)
     * @return Interpolated Vector2 point along the path, or null if path is empty
     */
    public Vector2 getPointAt(double t) {
        if (points.isEmpty()) {
            throw new IllegalStateException("Path contains no points.");
        }
        
        t = Math.max(0, Math.min(1, t));
        
        double indexDouble = t * (points.size() - 1);
        int index = (int) indexDouble;
        
        if (index >= points.size() - 1) {
            return points.get(points.size() - 1);
        }
        
        double fraction = indexDouble - index;
        Vector2 p1 = points.get(index);
        Vector2 p2 = points.get(index + 1);
        
        return p1.add(p2.sub(p1).mul(fraction));
    }

    /**
     * Gets the length along the path at parameter t in [0, 1].
     * 
     * @param t Fraction of the path (0 = start, 1 = end)
     * @return Length along the path at parameter t
     */
    public double getLengthAt(double t) {
        if (points.isEmpty()) {
            throw new IllegalStateException("Path contains no points.");
        }

        t = Math.max(0, Math.min(1, t));

        double indexDouble = t * (points.size() - 1);
        int index = (int) indexDouble;

        if (index >= points.size() - 1) {
            return totalLength;
        }

        double fraction = indexDouble - index;
        double lengthAtIndex = lengths.get(index);
        double segmentLength = lengths.get(index + 1) - lengths.get(index);

        return lengthAtIndex + segmentLength * fraction;
    }
    
    /**
     * Extracts a subpath from this path between two t values.
     * 
     * @param startT Start fraction of the path (0 = start)
     * @param endT End fraction of the path (1 = end)
     * @return A new RobotPath representing the subpath
     */
    public RobotPath getSubPath(double startT, double endT) {
        if (startT > endT) {
            throw new IllegalArgumentException("startT must be less than or equal to endT");
        }

        if(startT > 1 || endT < 0 || startT < 0 || endT > 1) {
            throw new IllegalArgumentException("startT and endT must be in the range [0, 1]");
        }

        startT = Math.max(0, Math.min(1, startT));
        endT = Math.max(0, Math.min(1, endT));
        
        int startIndex = (int) (startT * (points.size() - 1));
        int endIndex = (int) Math.ceil(endT * (points.size() - 1));
        
        endIndex = Math.min(endIndex, points.size() - 1);
        
        List<Vector2> subPoints = new ArrayList<>(points.subList(startIndex, endIndex + 1));
        List<Double> subCurvatures = new ArrayList<>(curvatures.subList(startIndex, endIndex + 1));
        return new RobotPath(subPoints, subCurvatures);
    }
    
    /**
     * Splits this path into multiple subpaths at the specified t values.
     * 
     * @param tValues List of t values in [0, 1] where the path should be split
     * @return A list of RobotPath objects representing the split segments
     */
    public List<RobotPath> split(List<Double> tValues) {
        List<RobotPath> paths = new ArrayList<>();

        tValues.sort(Double::compareTo);
        
        double lastT = 0;
        for (double t : tValues) {
            if (t < 0 || t > 1) {
                throw new IllegalArgumentException("t values must be in the range [0, 1]");
            }

            if (t > lastT && t <= 1.0) {
                paths.add(getSubPath(lastT, t));
                lastT = t;
            }
        }
        
        if (lastT < 1.0) {
            paths.add(getSubPath(lastT, 1.0));
        }
        
        return paths;
    }
    
    /**
     * Returns a copy of all points in this path.
     * 
     * @return List of Vector2 points defining the path
     */
    public List<Vector2> getPoints() {
        return new ArrayList<>(points);
    }
    
    /**
     * Gets the starting point of the path.
     * 
     * @return The first Vector2 point in the path, or null if empty
     */
    public Vector2 getStart() {
        return points.isEmpty() ? null : points.get(0);
    }
    
    /**
     * Gets the ending point of the path.
     * 
     * @return The last Vector2 point in the path, or null if empty
     */
    public Vector2 getEnd() {
        return points.isEmpty() ? null : points.get(points.size() - 1);
    }
    
    /**
     * Returns the number of points in the path.
     * 
     * @return The size of the points list
     */
    public int getPointCount() {
        return points.size();
    }

    /**
     * Returns the total length of the path.
     * 
     * @return Total length of the path
     */
    public double getTotalLength() {
        return totalLength;
    }

    public List<Double> getCurvatures() {
        return curvatures;
    }

    public List<Double> getLengths() {
        return lengths;
    }

}