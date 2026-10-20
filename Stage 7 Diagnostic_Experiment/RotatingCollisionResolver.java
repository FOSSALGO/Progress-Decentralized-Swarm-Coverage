package swarmdrone.decentralizedswarmcoverage_ijrcs;

import java.util.Arrays;

/** Simulator safety layer only. Never supplies world or peer states to drone planners. */
public final class RotatingCollisionResolver {
    private RotatingCollisionResolver() {}

    static int rank(int id, int timestep, int count) {
        return Math.floorMod(id - timestep, count);
    }
    private static boolean same(Point a, Point b) {
        return a.row() == b.row() && a.col() == b.col();
    }
    private static boolean moving(Point from, Point to) { return !same(from, to); }

    /** Return movements safe to execute simultaneously; rejected agents stay put. */
    public static Point[] resolve(Point[] from, Point[] proposed, int timestep) {
        int n = from.length;
        if (n != proposed.length) throw new IllegalArgumentException("Movement arrays differ");
        Point[] next = proposed.clone();
        for (int i = 0; i < n; i++) {
            if (from[i] == null || proposed[i] == null) throw new IllegalArgumentException("Null position");
            for (int j = 0; j < i; j++) if (same(from[i], from[j]))
                throw new IllegalArgumentException("Initial occupancy conflict");
        }
        // Conflicts are resolved until stable because rejecting one movement
        // can make its old cell unavailable to another drone.
        boolean changed;
        do {
            changed = false;
            for (int i = 0; i < n; i++) for (int j = i + 1; j < n; j++) {
                if (!conflict(from[i], next[i], from[j], next[j])) continue;
                boolean mi = moving(from[i], next[i]), mj = moving(from[j], next[j]);
                if (!mi && !mj) throw new IllegalStateException("Stationary drone collision");
                int loser;
                if (!mi) loser = j;
                else if (!mj) loser = i;
                else loser = rank(i, timestep, n) < rank(j, timestep, n) ? j : i;
                next[loser] = from[loser];
                changed = true;
            }
        } while (changed);
        return next;
    }

    /** Closed segments intersect: includes same destinations, swaps and diagonal X. */
    static boolean conflict(Point a, Point b, Point c, Point d) {
        if (same(b, d)) return true;
        if (!moving(a,b) && !moving(c,d)) return false;
        if (same(a,d) && same(c,b)) return true;
        // segment intersection in exact grid integer arithmetic (long for safety).
        long o1 = orient(a,b,c), o2 = orient(a,b,d);
        long o3 = orient(c,d,a), o4 = orient(c,d,b);
        if (o1 == 0 && onSegment(a,b,c)) return true;
        if (o2 == 0 && onSegment(a,b,d)) return true;
        if (o3 == 0 && onSegment(c,d,a)) return true;
        if (o4 == 0 && onSegment(c,d,b)) return true;
        return (o1 > 0) != (o2 > 0) && (o3 > 0) != (o4 > 0);
    }
    private static long orient(Point a, Point b, Point c) {
        return ((long)b.col()-a.col())*((long)c.row()-a.row())
             - ((long)b.row()-a.row())*((long)c.col()-a.col());
    }
    private static boolean onSegment(Point a, Point b, Point c) {
        return c.row() >= Math.min(a.row(),b.row()) && c.row() <= Math.max(a.row(),b.row())
            && c.col() >= Math.min(a.col(),b.col()) && c.col() <= Math.max(a.col(),b.col());
    }
}
