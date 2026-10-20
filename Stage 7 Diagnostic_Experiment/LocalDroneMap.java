package swarmdrone.decentralizedswarmcoverage_ijrcs;

/**
 * One drone's own occupancy observations. 0=UNKNOWN, 1=FREE, -1=OCCUPIED.
 * No ground-truth access, static world reference, or other agents' state.
 * The simulator supplies sensor observations through observe().
 */
public final class LocalDroneMap {
    public static final int UNKNOWN = 0;
    public static final int FREE = 1;
    public static final int OCCUPIED = -1;
    private final int[][] cells;
    private int knownCount;

    public LocalDroneMap(int rows, int cols) {
        if (rows <= 0 || cols <= 0) throw new IllegalArgumentException("Invalid map dimensions");
        cells = new int[rows][cols];
    }

    public int rows() { return cells.length; }
    public int cols() { return cells[0].length; }
    public int knownCount() { return knownCount; }
    public int get(int row, int col) { return cells[row][col]; }

    /** Returns true only if this observation changes previously unknown state. */
    public boolean observe(int row, int col, int state) {
        if (state != FREE && state != OCCUPIED) throw new IllegalArgumentException("Expected FREE or OCCUPIED");
        int old = cells[row][col];
        if (old == state) return false;
        if (old != UNKNOWN) throw new IllegalStateException("Conflicting occupancy observation at (" + row + "," + col + ")");
        cells[row][col] = state;
        knownCount++;
        return true;
    }

    /** Defensive snapshot for debug/tests; callers cannot modify the actual map. */
    public int[][] snapshot() {
        int[][] copy = new int[rows()][];
        for (int row = 0; row < rows(); row++) copy[row] = cells[row].clone();
        return copy;
    }
}
