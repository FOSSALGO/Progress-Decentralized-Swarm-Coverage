package swarmdrone.decentralizedswarmcoverage_ijrcs;
import java.util.ArrayList;
public class Drone {
    protected int initialRowPosition, initialColPosition, sensorFoV, frameIndex;
    protected ArrayList<Point> path;
    public Drone(int[][] m,int r,int c,int sensor,int cell) {
        initialRowPosition=r; initialColPosition=c; sensorFoV=sensor;
        path=new ArrayList<>();
    }
}
