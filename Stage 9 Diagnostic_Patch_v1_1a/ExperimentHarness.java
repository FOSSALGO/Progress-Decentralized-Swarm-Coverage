package swarmdrone.decentralizedswarmcoverage_ijrcs;
import java.util.*;
import java.nio.file.*;
import java.io.*;

public class ExperimentHarness {
  static record Scenario(int[][] grid, int[][] starts) {}
  static Scenario parse(Path file) throws IOException {
    List<String> s=Files.readAllLines(file);int k=0;
    if(s.get(k).startsWith("ENVIRONMENT"))k++;
    int r=Integer.parseInt(s.get(k++).split(":")[1].trim());
    int c=Integer.parseInt(s.get(k++).split(":")[1].trim());
    int[][] grid=new int[r][c];
    for(int i=0;i<r;i++){
       String[] parts=s.get(k++).split(":");
       if(parts.length!=c)throw new IOException("Invalid column count "+file);
       for(int j=0;j<c;j++)grid[i][j]=Integer.parseInt(parts[j].trim());
    }
    if(s.get(k).startsWith("DRONES"))k++;
    int n=Integer.parseInt(s.get(k++).split(":")[1].trim());
    int[][] starts=new int[n][2];
    for(int i=0;i<n;i++){String[] parts=s.get(k++).split(":");starts[i][0]=Integer.parseInt(parts[0].trim());starts[i][1]=Integer.parseInt(parts[1].trim());}
    return new Scenario(grid,starts);
  }
  public static void main(String[] args) throws Exception {
    try(BufferedWriter w=Files.newBufferedWriter(Path.of(args[1]))) {
      w.write("scenario,lambda,rho,deposit,drones,termination,steps,observed_percent,physical_coverage_percent,revisit_percent,overlap_percent,decisions,decision_changes,positive_path_decisions,penalty_spread_decisions,moves\n");
      for(Path p:Files.list(Path.of(args[0])).filter(x->x.toString().endsWith(".sc")).sorted().toList()) {
        for(double lambda:new double[]{0.0,0.5,1.0,2.0,4.0}){
          System.setProperty("swarm.lambda",Double.toString(lambda));
          Scenario s=parse(p); ArrayList<Drone> ds=new ArrayList<>();
          for(int[] start:s.starts)ds.add(new Drone(s.grid,start[0],start[1],4,12));
          DecentralizedExploration ex=new DecentralizedExploration(s.grid,ds);
          if(!Double.isFinite(ex.getObservedRatio()) || ex.getObservedRatio()>100.00001)throw new AssertionError("Bad observed ratio");
          if(ex.getSteps() <0 || ex.getSteps()>Integer.getInteger("swarm.maxSteps",2500))throw new AssertionError("Bad step count");
          for(Drone d:ds){
            if(d.path.size()!=ex.getSteps()+1)throw new AssertionError("Replay frames incompatible");
            for(Point pt:d.path)if(s.grid[pt.row()][pt.col()]==-1)throw new AssertionError("Drone entered obstacle");
          }
          for(int tick=0;tick<=ex.getSteps();tick++) {
            for(int a=0;a<ds.size();a++)for(int b=a+1;b<ds.size();b++){
              Point pa=ds.get(a).path.get(tick),pb=ds.get(b).path.get(tick);
              if(pa.equals(pb))throw new AssertionError("Drone occupancy collision at "+p+" tick="+tick+" pair="+a+","+b);
            }
          }
          w.write(String.format(Locale.ROOT,"%s,%.2f,0.05,1.0,%d,%s,%d,%.4f,%.4f,%.4f,%.4f,%d,%d,%d,%d,%d%n",
               p.getFileName(),lambda,ds.size(),ex.getTerminationReason(),ex.getSteps(),ex.getObservedRatio(),ex.getCoverage(),ex.getRevisit(),ex.getOverlap(),ex.getDecisions(),ex.getChangedDecisions(),ex.getPositivePathDecisions(),ex.getDifferingPenaltyDecisions(),ex.getSuccessfulMoves()));
        }
      }
    }
  }
}
