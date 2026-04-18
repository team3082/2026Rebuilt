package frc.robot.subsystems;
import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
public class VibeSight {
    private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private static NetworkTable table = inst.getTable("vibesight");
    private static DoubleArrayTopic arrTopic = inst.getDoubleArrayTopic("objects");
    static final DoubleArraySubscriber vibes = arrTopic.subscribe(new double[0]);
    public static void init(){
    }
    public static ArrayList<HashMap<String, Object>> fetch(){
        double[] data = vibes.get();
        ArrayList<HashMap<String, Object>> processedData = new ArrayList<>();
        for(int iter = 0; iter < data.length; iter += 6){
            HashMap<String, Object> boxData = new HashMap<>(); 
            boxData.put("classID", data[iter]);
            boxData.put("confidence", data[iter + 1]);
            boxData.put("point1", new double[] {data[iter + 2], data[iter + 3]});
            boxData.put("point2", new double[] {data[iter + 4], data[iter + 5]});
            processedData.add(boxData);
        }
        System.out.println(processedData);
        return processedData;

    }
}