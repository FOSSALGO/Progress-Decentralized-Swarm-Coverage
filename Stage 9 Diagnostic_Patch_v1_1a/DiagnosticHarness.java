package swarmdrone.decentralizedswarmcoverage_ijrcs;
import java.util.*;
import java.nio.file.*;
public class DiagnosticHarness {
  public static void main(String[] args) throws Exception {
    if(args.length<4)throw new IllegalArgumentException("Usage: scenario.sc lambda recovery output.csv");
    System.setProperty("swarm.lambda",args[1]);
    System.setProperty("swarm.recovery",args[2]);
    System.setProperty("swarm.trace",args[3]);
    ExperimentHarness.Scenario s=ExperimentHarness.parse(Path.of(args[0]));
    ArrayList<Drone> agents=new ArrayList<>();
    for(int[] start:s.starts())agents.add(new Drone(s.grid(),start[0],start[1],4,12));
    DecentralizedExploration sim=new DecentralizedExploration(s.grid(),agents);
    System.out.printf(Locale.ROOT,"CASE %s lambda=%s recovery=%s termination=%s steps=%d observed=%.4f rejects=%d yieldMoves=%d exhaustion=%d%n",
      args[0],args[1],args[2],sim.getTerminationReason(),sim.getSteps(),sim.getObservedRatio(),sim.getCollisionRejects(),sim.getAlternateMoves(),sim.getExhaustionEvents());
  }
}
