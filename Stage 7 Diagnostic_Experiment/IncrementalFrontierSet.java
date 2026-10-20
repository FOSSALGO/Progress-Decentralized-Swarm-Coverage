package swarmdrone.decentralizedswarmcoverage_ijrcs;

import java.util.BitSet;

/**
 * Incremental, 8-connected frontier tracker for a single local occupancy map.
 * A frontier is a known FREE cell with at least one in-bounds UNKNOWN neighbor.
 * Only changed occupancy cells and their 8-neighbors need re-evaluation.
 * No ground truth, global ownership, or other agents' data are accessed.
 */
public final class IncrementalFrontierSet {
    private final int rows, cols;
    private final BitSet frontier;
    private final BitSet dirty;
    private static final int[][] N8 = {
        {-1,-1}, {-1,0}, {-1,1}, {0,-1}, {0,1}, {1,-1}, {1,0}, {1,1}
    };

    public IncrementalFrontierSet(int rows, int cols) {
        if (rows <= 0 || cols <= 0) throw new IllegalArgumentException("Invalid dimensions");
        this.rows = rows;
        this.cols = cols;
        frontier = new BitSet(rows * cols);
        dirty = new BitSet(rows * cols);
    }

    /** Mark an actual occupancy change; neighboring frontier validity may change too. */
    public void changed(int row, int col) {
        if (!inside(row, col)) throw new IndexOutOfBoundsException();
        dirty.set(key(row, col));
        for (int[] d : N8) {
            int nr = row + d[0], nc = col + d[1];
            if (inside(nr, nc)) dirty.set(key(nr, nc));
        }
    }

    /** Re-evaluate only dirty cells and clear the dirty queue. */
    public void update(LocalDroneMap map) {
        if (map.rows() != rows || map.cols() != cols) throw new IllegalArgumentException("Map dimension mismatch");
        for (int k = dirty.nextSetBit(0); k >= 0; k = dirty.nextSetBit(k + 1)) {
            int r = k / cols, c = k % cols;
            frontier.set(k, isFrontier(map, r, c));
        }
        dirty.clear();
    }

    public boolean contains(int row, int col) {
        return inside(row, col) && frontier.get(key(row, col));
    }

    public int size() { return frontier.cardinality(); }

    /** Defensive snapshot, used by tests and debugging only. */
    public BitSet snapshot() { return (BitSet) frontier.clone(); }

    /** Independent full scan oracle; never used to select targets. */
    public static BitSet fullScan(LocalDroneMap map) {
        BitSet out = new BitSet(map.rows() * map.cols());
        for (int r = 0; r < map.rows(); r++)
            for (int c = 0; c < map.cols(); c++)
                if (isFrontier(map, r, c)) out.set(r * map.cols() + c);
        return out;
    }

    private static boolean isFrontier(LocalDroneMap map, int r, int c) {
        if (map.get(r, c) != LocalDroneMap.FREE) return false;
        for (int[] d : N8) {
            int nr = r + d[0], nc = c + d[1];
            if (nr >= 0 && nr < map.rows() && nc >= 0 && nc < map.cols()
                    && map.get(nr, nc) == LocalDroneMap.UNKNOWN) return true;
        }
        return false;
    }

    private boolean inside(int r, int c) { return r >= 0 && r < rows && c >= 0 && c < cols; }
    private int key(int r, int c) { return r * cols + c; }
}
