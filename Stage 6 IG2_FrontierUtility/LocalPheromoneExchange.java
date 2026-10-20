package swarmdrone.decentralizedswarmcoverage_ijrcs;

/** Full local pheromone exchange: synchronous snapshot, one hop per timestep.
 * MAX merging is idempotent; it does not add pheromone on reception.
 */
public final class LocalPheromoneExchange {
    private LocalPheromoneExchange() {}
    /** Number of strictly increased (agent,cell) values due to exchange. */
    public static int exchange(LocalPheromoneMap[] maps, int[] row, int[] col, double radius) {
        int n = maps.length;
        if (row.length != n || col.length != n) throw new IllegalArgumentException("Array lengths differ");
        if (!Double.isFinite(radius) || radius < 0) throw new IllegalArgumentException("Invalid radius");
        if (n == 0) return 0;
        int rows = maps[0].rows(), cols = maps[0].cols();
        double[][][] snapshots = new double[n][][];
        for (int i = 0; i < n; i++) {
            if (maps[i].rows() != rows || maps[i].cols() != cols)
                throw new IllegalArgumentException("Pheromone grid sizes differ");
            snapshots[i] = maps[i].snapshot();
        }
        int changed = 0;
        double maxDist2 = radius * radius;
        for (int i = 0; i < n; i++)
            for (int r = 0; r < rows; r++)
                for (int c = 0; c < cols; c++) {
                    double maximum = snapshots[i][r][c];
                    for (int j = 0; j < n; j++) {
                        if (i == j) continue;
                        double dr = (double)row[i] - row[j], dc = (double)col[i] - col[j];
                        if (dr*dr + dc*dc <= maxDist2)
                            maximum = Math.max(maximum, snapshots[j][r][c]);
                    }
                    if (maximum > snapshots[i][r][c]) changed++;
                    maps[i].mergeMax(r, c, maximum);
                }
        return changed;
    }
}
