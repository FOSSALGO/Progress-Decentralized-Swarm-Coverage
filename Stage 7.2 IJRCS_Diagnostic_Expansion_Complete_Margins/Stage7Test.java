package swarmdrone.decentralizedswarmcoverage_ijrcs;
import java.util.*;

/** Deterministic headless contract tests for the frozen Stage 7 rules. */
public final class Stage7Test {
    private static void check(boolean ok, String msg) { if (!ok) throw new AssertionError(msg); }
    private static Point p(int r,int c) { return new Point(r,c); }
    private static boolean same(Point a, Point b) { return a.equals(b); }
    private static void safe(Point[] before, Point[] after) {
        for (int i=0;i<after.length;i++)for(int j=0;j<i;j++){
            check(!same(after[i],after[j]),"End occupancy conflict");
            check(!RotatingCollisionResolver.conflict(before[i],after[i],before[j],after[j]),"Segment intersection");
        }
    }
    private static int[][] world(int size, boolean wall) {
        int[][] w=new int[size][size];
        for(int r=0;r<size;r++)for(int c=0;c<size;c++)
            if(r==0||c==0||r==size-1||c==size-1)w[r][c]=-1;
        if(wall)for(int r=3;r<size-3;r++)if(r!=size/2)w[r][size/2]=-1;
        return w;
    }
    private static DecentralizedExploration run(int[][] world, int[][] starts){
        ArrayList<Drone> ds=new ArrayList<>();
        for(int[] start:starts)ds.add(new Drone(world,start[0],start[1],3,10));
        return new DecentralizedExploration(world,ds);
    }
    public static void main(String[] args){
        // Rotating tie preference alternates across timesteps.
        Point[] from={p(2,1),p(2,3)}, toward={p(2,2),p(2,2)};
        Point[] t0=RotatingCollisionResolver.resolve(from,toward,0);
        Point[] t1=RotatingCollisionResolver.resolve(from,toward,1);
        safe(from,t0); safe(from,t1);
        check(t0[0].equals(p(2,2)) && t0[1].equals(from[1]),"Priority t0");
        check(t1[1].equals(p(2,2)) && t1[0].equals(from[0]),"Priority t1");
        // Swap forbidden, neither can move.
        Point[] swapped=RotatingCollisionResolver.resolve(from,new Point[]{from[1],from[0]},0);
        check(Arrays.equals(swapped,from),"Swaps must be rejected");
        // Diagonal crossing forbidden.
        Point[] diag={p(1,1),p(1,2)};
        Point[] dx=RotatingCollisionResolver.resolve(diag,new Point[]{p(2,2),p(2,1)},0);
        safe(diag,dx);
        check(same(dx[0],p(2,2)) && same(dx[1],diag[1]),"Crossing priority");
        // If blocked drone stays, cascade must also block approach to old cell.
        Point[] chain={p(2,1),p(2,2),p(2,3)};
        Point[] cr=RotatingCollisionResolver.resolve(chain,new Point[]{p(2,2),p(2,3),p(2,3)},0);
        safe(chain,cr);
        check(Arrays.equals(chain,cr),"Blocked chain must not overlap");
        // Stress: every result safe, including multiple contenders and diagonal paths.
        Random rng=new Random(90721);
        int[][] moves={{0,0},{-1,0},{1,0},{0,-1},{0,1},{-1,-1},{-1,1},{1,-1},{1,1}};
        for(int trial=0;trial<2000;trial++){
            Point[] a={p(5,5),p(5,6),p(6,5),p(6,6)};
            Point[] b=new Point[4];
            for(int i=0;i<4;i++){int[] delta=moves[rng.nextInt(moves.length)];b[i]=p(a[i].row()+delta[0],a[i].col()+delta[1]);}
            safe(a,RotatingCollisionResolver.resolve(a,b,trial));
        }
        int[][] w=world(18,false);
        DecentralizedExploration complete=run(w,new int[][]{{2,2},{15,15}});
        check(complete.getTerminationReason().equals("EXPLORATION_COMPLETE"),"Expected complete");
        check(complete.getObservedRatio()==100.0,"Observed complete");
        check(complete.getRevisit()>=0 && complete.getRevisit()<=100,"Revisit bounds");
        check(complete.getOverlap()>=0 && complete.getOverlap()<=100,"Overlap bounds");
        check(complete.getTargetReselections()>0,"No target selections logged");
        System.out.println("PASS Stage 7: rotating collision priorities, crossing/swap/cascade, 2000 stress cases, completion and metrics");
    }
}
