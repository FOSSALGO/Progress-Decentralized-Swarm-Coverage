package swarmdrone.decentralizedswarmcoverage_ijrcs;
import java.util.*;
/** Deterministic map runner for observational diagnostics. */
public final class DiagnosticExperiment {
    private static int[][] world(int rows,int cols,int scenario) {
        int[][] w=new int[rows][cols];
        for(int r=0;r<rows;r++)for(int c=0;c<cols;c++)
            if(r==0||c==0||r==rows-1||c==cols-1)w[r][c]=-1;
        if(scenario==1)for(int r=3;r<rows-3;r++)if(r!=rows/2)w[r][cols/2]=-1;
        if(scenario==2)for(int c=3;c<cols-3;c++)if(c!=cols/2)w[rows/2][c]=-1;
        if(scenario==3)for(int r=4;r<rows-4;r+=4)for(int c=4;c<cols-4;c+=5)w[r][c]=-1;
        return w;
    }
    public static void main(String[] args) {
        int scenario=Integer.parseInt(args[0]);
        if(scenario<0||scenario>3)throw new IllegalArgumentException("scenario 0..3");
        int[][] w=world(18,22,scenario);
        ArrayList<Drone> ds=new ArrayList<>();
        ds.add(new Drone(w,2,2,3,10));ds.add(new Drone(w,15,19,3,10));
        DecentralizedExploration ex=new DecentralizedExploration(w,ds);
        System.out.printf(Locale.ROOT,"RESULT scenario=%d method=%s status=%s steps=%d visited=%.8f revisit=%.8f overlap=%.8f observed=%.8f reselect=%d blocked=%d%n",
          scenario,ex.getFrontierMethod(),ex.getTerminationReason(),ex.getSteps(),ex.getCoverage(),ex.getRevisit(),ex.getOverlap(),ex.getObservedRatio(),ex.getTargetReselections(),ex.getBlockedMoves());
    }
}
