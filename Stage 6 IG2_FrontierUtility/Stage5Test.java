package swarmdrone.decentralizedswarmcoverage_ijrcs;

import java.util.*;

/** Standalone deterministic pheromone tests and end-to-end passive-mode regression. */
public final class Stage5Test {
    static void ok(boolean x, String name) { if (!x) throw new AssertionError(name); }
    static void eq(double x, double y, String name) {
        if (Math.abs(x-y) > 1e-10) throw new AssertionError(name + " actual="+x+" expected="+y);
    }
    public static void main(String[] args) {
        LocalPheromoneMap a = new LocalPheromoneMap(5, 8);
        eq(a.get(1,2), 0, "initial zero");
        a.deposit(1, 2, .4);
        eq(a.get(1,2), .4, "first entry deposit");
        a.deposit(1, 2, .4);
        eq(a.get(1,2), .8, "repeat entry deposit");
        a.deposit(1, 2, .4);
        eq(a.get(1,2), 1, "bounded at one");
        a.evaporate(.05);
        eq(a.get(1,2), .95, "evaporation");
        double[][] alias = a.snapshot(); alias[1][2]=0;
        eq(a.get(1,2), .95, "defensive snapshot");
        a.evaporate(1);
        eq(a.get(1,2), 0, "fully evaporated");
        // Chain A--B--C (distance 2 each), no A--C link.
        LocalPheromoneMap[] p = {new LocalPheromoneMap(4,6),new LocalPheromoneMap(4,6),new LocalPheromoneMap(4,6)};
        p[2].deposit(1,4,.8);
        int[] row={2,2,2}, col={0,2,4};
        ok(LocalPheromoneExchange.exchange(p,row,col,2)==1,"one-hop changes B only");
        eq(p[1].get(1,4),.8,"B hears C");
        eq(p[0].get(1,4),0,"A cannot hear C instantaneously");
        ok(LocalPheromoneExchange.exchange(p,row,col,2)==1,"next timestep reaches A");
        eq(p[0].get(1,4),.8,"A hears C via B next timestep");
        ok(LocalPheromoneExchange.exchange(p,row,col,2)==0,"repeated message no double counting");
        eq(p[1].get(1,4),.8,"bounded max, not sum");
        // Equal-time combination of two deposit sources: max, NOT sum.
        p[0].deposit(0,0,.4); p[1].deposit(0,0,.7);
        LocalPheromoneExchange.exchange(p,row,col,2);
        eq(p[0].get(0,0),.7,"max merge over deposits");
        // Disconnected: no propagation.
        LocalPheromoneMap[] isolated={new LocalPheromoneMap(3,3), new LocalPheromoneMap(3,3)};
        isolated[0].deposit(1,1,.9);
        ok(LocalPheromoneExchange.exchange(isolated,new int[]{0,2},new int[]{0,2},1)==0,"out of range");
        eq(isolated[1].get(1,1),0,"no leakage outside radius");
        // End-to-end local movement and passive pheromone: compare sensor/paths in RegressionTest.
        int[][] w = new int[10][10];
        for(int r=0;r<10;r++)for(int c=0;c<10;c++) if(r==0||c==0||r==9||c==9) w[r][c]=-1;
        ArrayList<Drone> ds=new ArrayList<>(); ds.add(new Drone(w,2,2,3,10)); ds.add(new Drone(w,7,7,3,10));
        DecentralizedExploration ex=new DecentralizedExploration(w,ds);
        ok(ex.getLocalPheromoneSnapshot(0).length==10,"simulator created local fields");
        ok(ex.getDepositAmount()==.4,"default deposit");
        ok(ex.getEvaporationRate()==.05,"default rho");
        System.out.println("PASS Stage 5: deposition, saturation, evaporation, snapshots, max merge, one hop, disconnection, simulator integration");
    }
}
