package swarmdrone.decentralizedswarmcoverage_ijrcs;
import java.util.*;
public final class RegressionTest {
    private static void check(boolean b,String name) {if(!b) throw new AssertionError(name);}
    private static int[][] world(int rows,int cols,int scenario) {
        int[][] w=new int[rows][cols];
        for(int r=0;r<rows;r++)for(int c=0;c<cols;c++)
          if(r==0||c==0||r==rows-1||c==cols-1)w[r][c]=-1;
        if(scenario==1) for(int r=3;r<rows-3;r++) if(r!=rows/2) w[r][cols/2]=-1;
        if(scenario==2) for(int c=3;c<cols-3;c++) if(c!=cols/2) w[rows/2][c]=-1;
        if(scenario==3)for(int r=4;r<rows-4;r+=4)for(int c=4;c<cols-4;c+=5)w[r][c]=-1;
        return w;
    }
    public static void main(String[] args) {
        for(int scenario=0;scenario<4;scenario++){
            int[][] w=world(18,22,scenario);
            ArrayList<Drone> ds=new ArrayList<>();
            ds.add(new Drone(w,2,2,3,10));
            ds.add(new Drone(w,15,19,3,10));
            DecentralizedExploration ex=new DecentralizedExploration(w,ds);

            check(ex.getCoverage()>=0,"coverage nonnegative");
            System.out.printf(Locale.ROOT,"SCENARIO %d mode %s status %s steps %d observed %.2f visited %.2f revisit %.2f overlap %.2f%n",scenario,ex.getNavigationMode(),ex.getTerminationReason(),ex.getSteps(),ex.getObservedRatio(),ex.getCoverage(),ex.getRevisit(),ex.getOverlap());
        }
    }
}
