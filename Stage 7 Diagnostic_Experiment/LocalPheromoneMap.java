package swarmdrone.decentralizedswarmcoverage_ijrcs;

/** One drone's independent, dimensionless repulsive pheromone field [0,1].
 * No access to environment truth or the exploration planner.
 */
public final class LocalPheromoneMap {
    private final double[][] values;
    public LocalPheromoneMap(int rows, int cols) {
        if (rows <= 0 || cols <= 0) throw new IllegalArgumentException("Invalid grid size");
        values = new double[rows][cols];
    }
    public int rows() { return values.length; }
    public int cols() { return values[0].length; }
    public double get(int r, int c) { return values[r][c]; }
    public void evaporate(double rho) {
        if (!Double.isFinite(rho) || rho < 0 || rho > 1)
            throw new IllegalArgumentException("rho must be between zero and one");
        for (double[] line : values)
            for (int c = 0; c < line.length; c++) line[c] *= 1.0 - rho;
    }
    /** Called once at initialization and once for every successful cell entry. */
    public void deposit(int r, int c, double amount) {
        if (!Double.isFinite(amount) || amount <= 0 || amount > 1)
            throw new IllegalArgumentException("deposit amount must be in (0,1]");
        values[r][c] = Math.min(1.0, values[r][c] + amount);
    }
    void mergeMax(int r, int c, double peerValue) {
        if (!Double.isFinite(peerValue) || peerValue < 0 || peerValue > 1)
            throw new IllegalArgumentException("Invalid peer pheromone");
        values[r][c] = Math.max(values[r][c], peerValue);
    }
    public double[][] snapshot() {
        double[][] copy = new double[rows()][];
        for (int r = 0; r < rows(); r++) copy[r] = values[r].clone();
        return copy;
    }
}
