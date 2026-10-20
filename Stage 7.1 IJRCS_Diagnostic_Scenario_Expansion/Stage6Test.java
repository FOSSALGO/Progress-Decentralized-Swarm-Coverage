package swarmdrone.decentralizedswarmcoverage_ijrcs;
import java.util.*;
public final class Stage6Test {
    private static void check(boolean ok, String message) {
        if (!ok) throw new AssertionError(message);
    }
    private static int[][] scene() {
        int[][] m = new int[15][17];
        for (int r=0;r<15;r++) for (int c=0;c<17;c++)
            if(r==0||r==14||c==0||c==16)m[r][c]=-1;
        return m;
    }
    public static void main(String[] args) {
        // Ranking changes only when repulsive pheromone is weighted.
        double aFU=DecentralizedExploration.frontierUtility(.9,.3,.8,0);
        double bFU=DecentralizedExploration.frontierUtility(.7,.4,.1,0);
        double aRP=DecentralizedExploration.frontierUtility(.9,.3,.8,1);
        double bRP=DecentralizedExploration.frontierUtility(.7,.4,.1,1);
        check(aFU>bFU && bRP>aRP,"Pheromone must be able to change ranking");
        for (double lambda: new double[]{0,.5,1,2}) {
            double u=DecentralizedExploration.frontierUtility(.8,.2,.4,lambda);
            check(Math.abs(u-(.6-lambda*.4))<1e-10,"Utility formula mismatch");
        }
        int[][] world=scene();
        ArrayList<Drone> drones=new ArrayList<>();
        drones.add(new Drone(world,2,2,3,10));
        drones.add(new Drone(world,12,14,3,10));
        DecentralizedExploration experiment=new DecentralizedExploration(world,drones);
        check(experiment.getObservedRatio()>=0,"Observed coverage must be valid");
        check(experiment.getLocalFrontierSnapshot(0).equals(experiment.getFullScanFrontierSnapshot(0)),"FTFD mismatch");
        check(experiment.getLocalFrontierSnapshot(1).equals(experiment.getFullScanFrontierSnapshot(1)),"FTFD mismatch");
        check(experiment.getExpectedGain(0,2,2)>=0,"Expected gain must be nonnegative");
        check(experiment.getFrontierPheromone(0,2,2)>=0 && experiment.getFrontierPheromone(0,2,2)<=1,"Pheromone out of range");
        System.out.println("PASS Stage 6: score, ranking, IG, FTFD, pheromone bounds");
    }
}
