package swarmdrone.decentralizedswarmcoverage_ijrcs;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.Locale;

/** Optional passive CSV instrumentation. No access to maps or planner state. */
final class FrontierDiagnostics implements AutoCloseable {
    private final PrintWriter out;
    private int decisions, changed, nonzero, peerTriggers, occupancyTriggers, replans;

    FrontierDiagnostics(String path) {
        if (path == null || path.isBlank()) { out = null; return; }
        try {
            Path p = Path.of(path);
            Path parent = p.toAbsolutePath().getParent();
            if (parent != null) Files.createDirectories(parent);
            out = new PrintWriter(Files.newBufferedWriter(p, StandardCharsets.UTF_8));
            out.println("event,timestep,drone,method,candidates,p_min,p_max,p_mean,p_sd,p_positive,fu_row,fu_col,rp_row,rp_col,actual_row,actual_col,fu_score,rp_score,lambda,occupancy_event,peer_pheromone_event,invalid_goal,short_route,old_goal_row,old_goal_col,map_nonzero,map_max,fu_margin,rp_margin,fu_winner_p,fu_second_p,fu_second_r,fu_second_c,rp_second_r,rp_second_c");
        } catch (IOException e) { throw new UncheckedIOException("Could not open diagnostic CSV", e); }
    }
    boolean enabled() { return out != null; }
    void replan(int t,int drone,String method,boolean occupancy,boolean peer,boolean invalid,boolean shortRoute,Point oldGoal) {
        if (!enabled()) return;
        replans++; if (occupancy) occupancyTriggers++; if (peer) peerTriggers++;
        out.printf(Locale.ROOT,"REPLAN,%d,%d,%s,,,,,,,,,,,,,,,,%b,%b,%b,%b,%d,%d,,%n",
                t,drone,method,occupancy,peer,invalid,shortRoute,
                oldGoal == null ? -1 : oldGoal.row(),oldGoal == null ? -1 : oldGoal.col());
    }
    void decision(int t,int drone,String method,int count,double min,double max,double mean,double sd,int positive,
                  int fuR,int fuC,int rpR,int rpC,int aR,int aC,double fuScore,double rpScore,double lambda,int mapNonzero,double mapMax, double fuMargin,double rpMargin,double fuWinnerP,double fuSecondP,int fuSecondR,int fuSecondC,int rpSecondR,int rpSecondC) {
        if (!enabled()) return;
        decisions++; if (positive > 0) nonzero++;
        if (fuR != rpR || fuC != rpC) changed++;
        out.printf(Locale.ROOT,"DECISION,%d,%d,%s,%d,%.9f,%.9f,%.9f,%.9f,%d,%d,%d,%d,%d,%d,%d,%.9f,%.9f,%.9f,,,,,,,%d,%.9f,%.9f,%.9f,%.9f,%.9f,%d,%d,%d,%d%n",
                t,drone,method,count,min,max,mean,sd,positive,fuR,fuC,rpR,rpC,aR,aC,fuScore,rpScore,lambda,mapNonzero,mapMax,fuMargin,rpMargin,fuWinnerP,fuSecondP,fuSecondR,fuSecondC,rpSecondR,rpSecondC);
    }
    @Override public void close() {
        if (!enabled()) return;
        out.close();
    }
}
