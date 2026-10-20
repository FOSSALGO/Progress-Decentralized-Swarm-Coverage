package swarmdrone.decentralizedswarmcoverage_ijrcs;

/**
 * Stage 4 occupancy-only peer exchange. Each drone reads its neighbors' maps
 * from a single immutable pre-exchange snapshot; updates propagate one hop
 * per call. No access to simulator truth, target selection, or pheromone.
 */
public final class LocalMapExchange {
    private LocalMapExchange() {}

    /** Returns number of new cells learned by all agents. */
    public static int exchange(LocalDroneMap[] maps, int[] row, int[] col,
                               double communicationRadius, IncrementalFrontierSet[] frontiers) {
        int n = maps.length;
        if (row.length != n || col.length != n || frontiers.length != n)
            throw new IllegalArgumentException("Drone-array lengths differ");
        if (!Double.isFinite(communicationRadius) || communicationRadius < 0)
            throw new IllegalArgumentException("Invalid communication radius");
        if (n == 0) return 0;
        int rows = maps[0].rows(), cols = maps[0].cols();
        int[][][] snapshot = new int[n][][];
        for (int i = 0; i < n; i++) {
            if (maps[i].rows() != rows || maps[i].cols() != cols)
                throw new IllegalArgumentException("Local map dimensions differ");
            snapshot[i] = maps[i].snapshot();
        }
        int learned = 0;
        double radiusSquared = communicationRadius * communicationRadius;
        for (int i = 0; i < n; i++) {
            for (int r = 0; r < rows; r++) {
                for (int c = 0; c < cols; c++) {
                    int incoming = LocalDroneMap.UNKNOWN;
                    for (int j = 0; j < n; j++) {
                        if (j == i) continue;
                        double dr = (double) row[i] - row[j];
                        double dc = (double) col[i] - col[j];
                        if (dr * dr + dc * dc > radiusSquared) continue;
                        int state = snapshot[j][r][c];
                        if (state == LocalDroneMap.UNKNOWN) continue;
                        if (incoming != LocalDroneMap.UNKNOWN && incoming != state)
                            throw new IllegalStateException("Conflicting peer occupancy at (" + r + "," + c + ")");
                        incoming = state;
                    }
                    if (incoming == LocalDroneMap.UNKNOWN) continue;
                    if (maps[i].get(r, c) == LocalDroneMap.UNKNOWN) {
                        maps[i].observe(r, c, incoming);
                        frontiers[i].changed(r, c);
                        learned++;
                    } else if (maps[i].get(r, c) != incoming) {
                        throw new IllegalStateException("Conflicting received occupancy at (" + r + "," + c + ")");
                    }
                }
            }
        }
        return learned;
    }
}
