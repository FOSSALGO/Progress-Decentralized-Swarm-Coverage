package swarmdrone.decentralizedswarmcoverage_ijrcs;
import java.util.*;
/** Prespecified, deterministic experimental scenario generator; does not modify the algorithm. */
public final class ExpansionExperiment {
    static int[][] world(int rows,int cols,String layout,long seed) {
        int[][] w=new int[rows][cols];
        for(int r=0;r<rows;r++)for(int c=0;c<cols;c++)if(r==0||c==0||r==rows-1||c==cols-1)w[r][c]=-1;
        Random rng=new Random(seed);
        switch(layout) {
            case "open": break;
            case "clutter":
                for(int r=3;r<rows-3;r++)for(int c=3;c<cols-3;c++)
                    if(rng.nextDouble()<0.12)w[r][c]=-1;
                break;
            case "rooms":
                for(int r=rows/3;r<=2*rows/3;r+=rows/3) {
                    int door=3+rng.nextInt(cols-6);
                    for(int c=2;c<cols-2;c++)if(Math.abs(c-door)>1)w[r][c]=-1;
                }
                for(int c=cols/3;c<=2*cols/3;c+=cols/3) {
                    int door=3+rng.nextInt(rows-6);
                    for(int r=2;r<rows-2;r++)if(Math.abs(r-door)>1)w[r][c]=-1;
                }
                break;
            default: throw new IllegalArgumentException(layout);
        }
        // Reserve fixed start neighborhoods; independent of layout and seed.
        for(int[] p:new int[][]{{2,2},{rows-3,cols-3},{2,cols-3},{rows-3,2}})
            for(int dr=-1;dr<=1;dr++)for(int dc=-1;dc<=1;dc++)w[p[0]+dr][p[1]+dc]=0;
        return w;
    }
    public static void main(String[] args) {
        if(args.length!=5)throw new IllegalArgumentException("rows cols layout drones seed");
        int rows=Integer.parseInt(args[0]),cols=Integer.parseInt(args[1]);
        String layout=args[2];int n=Integer.parseInt(args[3]);long seed=Long.parseLong(args[4]);
        if(n!=2&&n!=4)throw new IllegalArgumentException("2 or 4 drones");
        int[][] w=world(rows,cols,layout,seed);
        int[][] starts={{2,2},{rows-3,cols-3},{2,cols-3},{rows-3,2}};
        ArrayList<Drone> drones=new ArrayList<>();
        for(int i=0;i<n;i++)drones.add(new Drone(w,starts[i][0],starts[i][1],8,10));
        DecentralizedExploration ex=new DecentralizedExploration(w,drones);
        System.out.printf(Locale.ROOT,"EXPERIMENT,size=%dx%d,layout=%s,drones=%d,seed=%d,method=%s,status=%s,steps=%d,observed=%.6f,visited=%.6f,revisit=%.6f,overlap=%.6f,replans=%d,blocked=%d%n",rows,cols,layout,n,seed,ex.getFrontierMethod(),ex.getTerminationReason(),ex.getSteps(),ex.getObservedRatio(),ex.getCoverage(),ex.getRevisit(),ex.getOverlap(),ex.getTargetReselections(),ex.getBlockedMoves());
    }
}
