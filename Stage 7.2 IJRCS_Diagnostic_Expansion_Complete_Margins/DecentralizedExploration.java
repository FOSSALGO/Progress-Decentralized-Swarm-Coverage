package swarmdrone.decentralizedswarmcoverage_ijrcs;

import java.util.*;

/**
 * Stage 7 integration of three-method frontier scoring with preserved legacy benchmark: legacy exploration benchmark with independent
 * per-drone sensor maps recorded alongside the legacy global decision map.
 * Legacy decisions intentionally retain a single aggregate occupancy map,
 * exclusive ownership of detected frontier cells, and nearest-frontier travel.
 * The original program gave path planning access to ground-truth base_map; this
 * benchmark deliberately retains that oracle navigation assumption to make a
 * fair conceptual comparison with the uploaded legacy algorithm. It is NOT a
 * decentralized algorithm and does NOT implement repulsive pheromone.
 * <p>
 * Changes from legacy: A* with Euclidean edge lengths instead of BFS step count,
 * simultaneous one-cell movement, collision resolution, and replay-compatible
 * observation timestamps. No corner-cutting through obstacles.
 */
public final class DecentralizedExploration {
    /** Default is legacy for unchanged UI; -Dswarm.navigation=local enables Stage 2. */
    private static final boolean LOCAL_NAVIGATION =
            "local".equalsIgnoreCase(System.getProperty("swarm.navigation", "legacy"));
    private static final String FRONTIER_METHOD = System.getProperty("swarm.frontier", "nearest").toLowerCase(Locale.ROOT);
    private static final boolean VERIFY_FTFD = Boolean.getBoolean("swarm.verifyFtfd");
    private static final int MAX_STEPS = Integer.getInteger("swarm.legacy.maxSteps", 2500);
    private static final int[][] N8 = {{0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}};
    private static final double SQRT2 = Math.sqrt(2.0);
    private final int[][] truth;
    private final ArrayList<Drone> drones;
    private final int rows, cols;
    private final int[][] map; // LEGACY aggregate, retained only for compatible frontier decisions
    private final LocalDroneMap[] localMaps; // independently sensed, no sharing
    private final LocalPheromoneMap[] localPheromones; // Stage 5, not used for navigation
    private final double evaporationRate, depositAmount, pheromoneWeight;
    private final int[] lastKnownCount;
    private final boolean[] needsSelection;
    private int sharedPheromoneUpdates;
    private int blockedMoves, targetReselections;
    // Read-only diagnostic observer; no algorithmic decisions depend on it.
    private final FrontierDiagnostics diagnostics;
    private final int stallLimit = Integer.getInteger("swarm.stallSteps", 30);
    private final IncrementalFrontierSet[] localFrontiers; // Stage 3 incremental FTFD
    private final double communicationRadius; // Stage 4; local mode only
    private int receivedCellUpdates;
    private final int[][] firstObservedAt;
    private final int[][] visits;
    private final BitSet[] visitors;
    private final boolean[][] reachable;
    private final int[][] owner; // per-cell exclusive frontier owner, -1 if unassigned
    private final Point[] goals;
    private final List<Point>[] routes;
    private final int[] row, col;
    private final int reachableFree;
    private final double coverage;
    private final double revisit;
    private final double overlap;
    private int observedReachable;
    private int steps;
    private int uniqueVisits;
    private int totalVisits;
    private String terminationReason = "NOT_STARTED";

    @SuppressWarnings("unchecked")
    public DecentralizedExploration(int[][] environment, ArrayList<Drone> drones) {
        if (environment == null || environment.length == 0 || environment[0].length == 0 || drones == null || drones.isEmpty())
            throw new IllegalArgumentException("INVALID_SETUP: world and at least one drone required");
        this.truth = environment;
        this.drones = drones;
        diagnostics = new FrontierDiagnostics(System.getProperty("swarm.diagnostic.csv", ""));
        rows = environment.length;
        cols = environment[0].length;
        map = new int[rows][cols];
        localMaps = new LocalDroneMap[drones.size()];
        localFrontiers = new IncrementalFrontierSet[drones.size()];
        localPheromones = new LocalPheromoneMap[drones.size()];
        evaporationRate = Double.parseDouble(System.getProperty("swarm.pheromone.rho", "0.05"));
        depositAmount = Double.parseDouble(System.getProperty("swarm.pheromone.deposit", "0.4"));
        pheromoneWeight = Double.parseDouble(System.getProperty("swarm.pheromone.lambda", "1.0"));
        if (stallLimit <= 0 || MAX_STEPS <= 0) throw new IllegalArgumentException("Invalid timestep limits");
        if (!Set.of("nearest", "utility", "pheromone").contains(FRONTIER_METHOD))
            throw new IllegalArgumentException("Invalid swarm.frontier: " + FRONTIER_METHOD);
        if (!Double.isFinite(pheromoneWeight) || pheromoneWeight < 0)
            throw new IllegalArgumentException("Invalid pheromone weight");
        lastKnownCount = new int[drones.size()];
        needsSelection = new boolean[drones.size()];
        Arrays.fill(needsSelection, true);
        if (!Double.isFinite(evaporationRate) || evaporationRate < 0 || evaporationRate > 1)
            throw new IllegalArgumentException("Invalid pheromone evaporation rate");
        if (!Double.isFinite(depositAmount) || depositAmount <= 0 || depositAmount > 1)
            throw new IllegalArgumentException("Invalid pheromone deposit amount");
        for (int i = 0; i < localMaps.length; i++) {
            localMaps[i] = new LocalDroneMap(rows, cols);
            localPheromones[i] = new LocalPheromoneMap(rows, cols);
            localFrontiers[i] = new IncrementalFrontierSet(rows, cols);
        }
        int defaultRadius = drones.stream().mapToInt(d -> d.sensorFoV).min().orElse(0) * 2;
        communicationRadius = Double.parseDouble(System.getProperty("swarm.communicationRadius", Integer.toString(defaultRadius)));
        if (!Double.isFinite(communicationRadius) || communicationRadius < 0)
            throw new IllegalArgumentException("Invalid communication radius");
        owner = new int[rows][cols];
        firstObservedAt = new int[rows][cols];
        visits = new int[rows][cols];
        visitors = new BitSet[rows * cols];
        reachable = new boolean[rows][cols];
        goals = new Point[drones.size()];
        routes = new List[drones.size()];
        row = new int[drones.size()];
        col = new int[drones.size()];
        for (int r = 0; r < rows; r++) {
            Arrays.fill(firstObservedAt[r], -1);
            Arrays.fill(owner[r], -1);
        }
        for (int i = 0; i < drones.size(); i++) {
            Drone d = drones.get(i);
            row[i] = d.initialRowPosition;
            col[i] = d.initialColPosition;
            if (!free(row[i], col[i]))
                throw new IllegalArgumentException("Drone start is not a free cell: D" + (i + 1));
            for (int j = 0; j < i; j++)
                if (row[j] == row[i] && col[j] == col[i])
                    throw new IllegalArgumentException("Drone initial cells must be distinct");
            d.path = new ArrayList<>();
            d.path.add(new Point(row[i], col[i]));
            d.frameIndex = 0;
            routes[i] = new ArrayList<>();
            visit(i, row[i], col[i]);
            // Exactly one initial-cell deposit, local mode only.
            if (LOCAL_NAVIGATION) localPheromones[i].deposit(row[i], col[i], depositAmount);
        }
        reachableFree = markReachable();
        simulate();
        coverage = reachableFree == 0 ? 0 : 100.0 * uniqueVisits / reachableFree;
        revisit = totalVisits == 0 ? 0 : 100.0 * (totalVisits - uniqueVisits) / totalVisits;
        int overlapCells = 0;
        for (BitSet b : visitors) if (b != null && b.cardinality() > 1) overlapCells++;
        overlap = uniqueVisits == 0 ? 0 : 100.0 * overlapCells / uniqueVisits;
        diagnostics.close();
        System.out.printf(Locale.ROOT,
                (LOCAL_NAVIGATION ? "Stage 7 local FTFD + " + FRONTIER_METHOD + " frontier:" : "Legacy A* benchmark:") + " %s; steps=%d observed=%.2f%% visited=%.2f%% revisit=%.2f%% overlap=%.2f%%%n",
                terminationReason, steps, getObservedRatio(), coverage, revisit, overlap);
    }

    private boolean inside(int r, int c) {
        return r >= 0 && r < rows && c >= 0 && c < cols;
    }

    private boolean free(int r, int c) {
        return inside(r, c) && truth[r][c] != -1;
    }

    private boolean legal(int r, int c, int nr, int nc) {
        if (!free(nr, nc)) return false;
        int dr = nr - r, dc = nc - c;
        return dr == 0 || dc == 0 || (free(r, nc) && free(nr, c));
    }

    private int markReachable() {
        ArrayDeque<Point> q = new ArrayDeque<>();
        for (int i = 0; i < row.length; i++)
            if (!reachable[row[i]][col[i]]) {
                reachable[row[i]][col[i]] = true;
                q.add(new Point(row[i], col[i]));
            }
        int count = 0;
        while (!q.isEmpty()) {
            Point p = q.poll();
            count++;
            int r = p.row(), c = p.col();
            for (int[] d : N8) {
                int nr = r + d[0], nc = c + d[1];
                if (legal(r, c, nr, nc) && !reachable[nr][nc]) {
                    reachable[nr][nc] = true;
                    q.add(new Point(nr, nc));
                }
            }
        }
        return count;
    }

    private void visit(int id, int r, int c) {
        totalVisits++;
        if (visits[r][c]++ == 0) uniqueVisits++;
        int k = r * cols + c;
        if (visitors[k] == null) visitors[k] = new BitSet(drones.size());
        visitors[k].set(id);
    }

    private void discover(int id, int r, int c, int t) {
        if (!inside(r, c)) return;
        // Only this drone receives the physical observation.
        if (localMaps[id].observe(r, c, truth[r][c] == -1 ? LocalDroneMap.OCCUPIED : LocalDroneMap.FREE))
            localFrontiers[id].changed(r, c);
        if (!inside(r, c)) return;
        if (map[r][c] == 0) {
            map[r][c] = truth[r][c] == -1 ? -1 : 1;
            if (map[r][c] == 1 && reachable[r][c]) {
                firstObservedAt[r][c] = t;
                observedReachable++;
            }
        }
    }

    /**
     * Sensor observation: 360 radial rays, obstacles block the field of view.
     */
    private void sense(int id, int r, int c, int radius, int t) {
        discover(id, r, c, t);
        for (int degree = 0; degree < 360; degree++) {
            double theta = Math.toRadians(degree), dr = Math.sin(theta), dc = Math.cos(theta);
            double x = c + 0.5, y = r + 0.5;
            int prev = -1;
            for (double distance = 0.125; distance <= radius + 0.5; distance += 0.125) {
                int rr = (int) Math.floor(y + dr * distance), cc = (int) Math.floor(x + dc * distance);
                if (!inside(rr, cc)) break;
                int key = rr * cols + cc;
                if (key == prev) continue;
                prev = key;
                discover(id, rr, cc, t);
                if (truth[rr][cc] == -1) break;
            }
        }
    }

    /**
     * Old simulation's frontier uniqueness: a frontier belongs to the first
     * agent that detects it, until it is explored. Unlike a claim lease, this
     * is globally coordinated ownership, matching the legacy reference.
     */
    private void detectOwnedFrontiers() {
        // A known-free cell touching unknown is a standard reachable frontier.
        for (int i = 0; i < drones.size(); i++) {
            int radius = drones.get(i).sensorFoV;
            for (int r = Math.max(0, row[i] - radius); r <= Math.min(rows - 1, row[i] + radius); r++)
                for (int c = Math.max(0, col[i] - radius); c <= Math.min(cols - 1, col[i] + radius); c++) {
                    if (owner[r][c] != -1 || map[r][c] != 1) continue;
                    if (Math.hypot(r - row[i], c - col[i]) > radius + 0.5) continue;
                    if (isFrontier(r, c)) owner[r][c] = i;
                }
        }
        // Once it is no longer a frontier, allow another frontier to be assigned.
        for (int r = 0; r < rows; r++)
            for (int c = 0; c < cols; c++) if (owner[r][c] >= 0 && !isFrontier(r, c)) owner[r][c] = -1;
    }

    private boolean isFrontier(int r, int c) {
        if (map[r][c] != 1) return false;
        for (int[] v : N8) {
            int nr = r + v[0], nc = c + v[1];
            if (inside(nr, nc) && map[nr][nc] == 0) return true;
        }
        return false;
    }

    /**
     * A* to nearest owned frontier, using geometric distances, not hop count.
     * The heuristic is the minimum octile distance to any owned candidate,
     * so the first popped frontier is the globally shortest reachable one.
     */
    private List<Point> astarNearest(int id) {
        boolean[][] goalsMask = new boolean[rows][cols];
        ArrayList<Point> targets = new ArrayList<>();
        for (int r = 0; r < rows; r++)
            for (int c = 0; c < cols; c++)
                if (owner[r][c] == id && isFrontier(r, c)) {
                    goalsMask[r][c] = true;
                    targets.add(new Point(r, c));
                }
        if (targets.isEmpty()) return Collections.emptyList();
        double[][] best = new double[rows][cols];
        for (double[] a : best) Arrays.fill(a, Double.POSITIVE_INFINITY);
        PriorityQueue<Node> q = new PriorityQueue<>(Comparator.comparingDouble((Node n) -> n.f).thenComparingDouble(n -> n.g).thenComparingInt(n -> n.r).thenComparingInt(n -> n.c));
        int sr = row[id], sc = col[id];
        best[sr][sc] = 0;
        q.add(new Node(sr, sc, 0, heuristic(sr, sc, targets), null));
        while (!q.isEmpty()) {
            Node p = q.poll();
            if (p.g > best[p.r][p.c] + 1e-9) continue;
            if (goalsMask[p.r][p.c]) {
                LinkedList<Point> result = new LinkedList<>();
                for (Node n = p; n != null; n = n.parent) result.addFirst(new Point(n.r, n.c));
                return result;
            }
            for (int[] v : N8) {
                int nr = p.r + v[0], nc = p.c + v[1];
                // LEGACY ORACLE: uses true environment for safe paths, just as
                // old base_map. This must not be reported as local-map planning.
                if (!legal(p.r, p.c, nr, nc)) continue;
                double ng = p.g + (v[0] == 0 || v[1] == 0 ? 1 : SQRT2);
                if (ng + 1e-9 < best[nr][nc]) {
                    best[nr][nc] = ng;
                    q.add(new Node(nr, nc, ng, ng + heuristic(nr, nc, targets), p));
                }
            }
        }
        return Collections.emptyList();
    }

    private double heuristic(int r, int c, List<Point> targets) {
        double h = Double.POSITIVE_INFINITY;
        for (Point p : targets) {
            int dr = Math.abs(r - p.row()), dc = Math.abs(c - p.col());
            h = Math.min(h, Math.max(dr, dc) + (SQRT2 - 1) * Math.min(dr, dc));
        }
        return h;
    }

    /** Stage 2 preserved: only this drone's observations determine traversability. */
    private boolean localLegal(int id, int r, int c, int nr, int nc) {
        LocalDroneMap m = localMaps[id];
        if (!inside(nr, nc) || m.get(nr, nc) != LocalDroneMap.FREE) return false;
        int dr = nr - r, dc = nc - c;
        return dr == 0 || dc == 0 ||
                (m.get(r, nc) == LocalDroneMap.FREE && m.get(nr, c) == LocalDroneMap.FREE);
    }

    /** Stage 3: O(1) lookup into the incremental frontier set of this drone. */
    private boolean localFrontier(int id, int r, int c) {
        return localFrontiers[id].contains(r, c);
    }

    /** Test/debug snapshot: this drone's incremental frontier set (no mutable alias). */
    public BitSet getLocalFrontierSnapshot(int droneId) {
        return localFrontiers[droneId].snapshot();
    }

    /** Independent full-scan oracle for validation only; never used for planning. */
    public BitSet getFullScanFrontierSnapshot(int droneId) {
        return IncrementalFrontierSet.fullScan(localMaps[droneId]);
    }

    /** Dijkstra uses only the caller's local map; no truth, aggregate map or owner. */
    private List<Point> localShortestFrontier(int id) {
        double[][] best = new double[rows][cols];
        for (double[] line : best) Arrays.fill(line, Double.POSITIVE_INFINITY);
        PriorityQueue<Node> queue = new PriorityQueue<>(Comparator
                .comparingDouble((Node n) -> n.g)
                .thenComparingInt(n -> n.r).thenComparingInt(n -> n.c));
        int sr = row[id], sc = col[id];
        best[sr][sc] = 0;
        queue.add(new Node(sr, sc, 0, 0, null));
        while (!queue.isEmpty()) {
            Node current = queue.poll();
            if (current.g > best[current.r][current.c] + 1e-9) continue;
            if (localFrontier(id, current.r, current.c)) {
                LinkedList<Point> route = new LinkedList<>();
                for (Node n = current; n != null; n = n.parent)
                    route.addFirst(new Point(n.r, n.c));
                return route;
            }
            for (int[] d : N8) {
                int nr = current.r + d[0], nc = current.c + d[1];
                if (!localLegal(id, current.r, current.c, nr, nc)) continue;
                double nd = current.g + ((d[0] == 0 || d[1] == 0) ? 1 : SQRT2);
                if (nd + 1e-9 < best[nr][nc]) {
                    best[nr][nc] = nd;
                    Node next = new Node(nr, nc, nd, nd, current);
                    queue.add(next);
                }
            }
        }
        return Collections.emptyList();
    }

    /**
     * Stage 6: one Dijkstra expansion supplies distances and predecessors for
     * ALL reachable frontiers. No truth, global ownership or shared map.
     */
    private List<Point> scoredLocalFrontier(int id) {
        double[][] dist = new double[rows][cols];
        Node[][] parent = new Node[rows][cols];
        for (double[] line : dist) Arrays.fill(line, Double.POSITIVE_INFINITY);
        PriorityQueue<Node> queue = new PriorityQueue<>(Comparator
                .comparingDouble((Node n) -> n.g)
                .thenComparingInt(n -> n.r).thenComparingInt(n -> n.c));
        int sr = row[id], sc = col[id];
        dist[sr][sc] = 0;
        queue.add(new Node(sr, sc, 0, 0, null));
        while (!queue.isEmpty()) {
            Node node = queue.poll();
            if (node.g > dist[node.r][node.c] + 1e-9) continue;
            for (int[] direction : N8) {
                int nr = node.r + direction[0], nc = node.c + direction[1];
                if (!localLegal(id, node.r, node.c, nr, nc)) continue;
                double edge = direction[0] == 0 || direction[1] == 0 ? 1 : SQRT2;
                double candidate = node.g + edge;
                if (candidate + 1e-9 < dist[nr][nc]) {
                    dist[nr][nc] = candidate;
                    parent[nr][nc] = node;
                    queue.add(new Node(nr, nc, candidate, candidate, node));
                }
            }
        }
        BitSet frontierBits = localFrontiers[id].snapshot();
        ArrayList<FrontierScore> candidates = new ArrayList<>();
        double maxGain = 0, maxDistance = 0;
        for (int key = frontierBits.nextSetBit(0); key >= 0; key = frontierBits.nextSetBit(key + 1)) {
            int r = key / cols, c = key % cols;
            if (!Double.isFinite(dist[r][c])) continue;
            int gain = expectedSensingGain(id, r, c);
            double pheromone = meanFrontierPheromone(id, r, c);
            candidates.add(new FrontierScore(r, c, gain, dist[r][c], pheromone));
            maxGain = Math.max(maxGain, gain);
            maxDistance = Math.max(maxDistance, dist[r][c]);
        }
        if (candidates.isEmpty()) return Collections.emptyList();
        FrontierScore chosen = null;
        double bestUtility = Double.NEGATIVE_INFINITY;
        for (FrontierScore f : candidates) {
            double gain = maxGain == 0 ? 0 : f.gain / maxGain;
            double distance = maxDistance == 0 ? 0 : f.distance / maxDistance;
            double utility = frontierUtility(gain, distance, f.pheromone,
                    "pheromone".equals(FRONTIER_METHOD) ? pheromoneWeight : 0);
            if (chosen == null || utility > bestUtility + 1e-9 ||
                    (Math.abs(utility - bestUtility) <= 1e-9 &&
                            (f.distance < chosen.distance - 1e-9 ||
                                    (Math.abs(f.distance - chosen.distance) <= 1e-9 &&
                                            (f.r * cols + f.c < chosen.r * cols + chosen.c))))) {
                chosen = f;
                bestUtility = utility;
            }
        }
        // Diagnostic counterfactual: score the SAME frontier candidates under
        // FU and FU-RP; neither counterfactual changes the actual chosen target.
        if (diagnostics.enabled()) {
            FrontierScore fu = null, rp = null;
            double fuBest = Double.NEGATIVE_INFINITY, rpBest = Double.NEGATIVE_INFINITY;
            double pMin = Double.POSITIVE_INFINITY, pMax = Double.NEGATIVE_INFINITY, pSum = 0, pSquare = 0;
            int positive = 0;
            for (FrontierScore f : candidates) {
                double gn = maxGain == 0 ? 0 : f.gain / maxGain;
                double dn = maxDistance == 0 ? 0 : f.distance / maxDistance;
                double uf = frontierUtility(gn, dn, f.pheromone, 0);
                double ur = frontierUtility(gn, dn, f.pheromone, pheromoneWeight);
                if (betterDiagnostic(f, uf, fu, fuBest)) { fu = f; fuBest = uf; }
                if (betterDiagnostic(f, ur, rp, rpBest)) { rp = f; rpBest = ur; }
                pMin = Math.min(pMin, f.pheromone); pMax = Math.max(pMax, f.pheromone);
                pSum += f.pheromone; pSquare += f.pheromone * f.pheromone;
                if (f.pheromone > 1e-12) positive++;
            }
            // Passive margins: best minus runner-up, with the same score and
            // deterministic tie-breaker used by the planner. Never selects a route.
            FrontierScore fuSecond = null, rpSecond = null;
            double fuSecondScore = Double.NEGATIVE_INFINITY, rpSecondScore = Double.NEGATIVE_INFINITY;
            for (FrontierScore f : candidates) {
                if (f != fu) {
                    double gn = maxGain == 0 ? 0 : f.gain / maxGain;
                    double dn = maxDistance == 0 ? 0 : f.distance / maxDistance;
                    double score = frontierUtility(gn, dn, f.pheromone, 0);
                    if (betterDiagnostic(f, score, fuSecond, fuSecondScore)) {
                        fuSecond = f; fuSecondScore = score;
                    }
                }
                if (f != rp) {
                    double gn = maxGain == 0 ? 0 : f.gain / maxGain;
                    double dn = maxDistance == 0 ? 0 : f.distance / maxDistance;
                    double score = frontierUtility(gn, dn, f.pheromone, pheromoneWeight);
                    if (betterDiagnostic(f, score, rpSecond, rpSecondScore)) {
                        rpSecond = f; rpSecondScore = score;
                    }
                }
            }
            double fuMargin = fuSecond == null ? Double.NaN : fuBest - fuSecondScore;
            double rpMargin = rpSecond == null ? Double.NaN : rpBest - rpSecondScore;
            int mapNonzero = 0;
            double mapMax = 0;
            for (double[] line : localPheromones[id].snapshot())
                for (double value : line) {
                    if (value > 1e-12) mapNonzero++;
                    mapMax = Math.max(mapMax, value);
                }
            double avg = pSum / candidates.size();
            double deviation = Math.sqrt(Math.max(0, pSquare / candidates.size() - avg * avg));
            diagnostics.decision(currentDiagnosticStep, id, FRONTIER_METHOD, candidates.size(),
                    pMin, pMax, avg, deviation, positive,
                    fu.r, fu.c, rp.r, rp.c, chosen.r, chosen.c, fuBest, rpBest,
                    pheromoneWeight, mapNonzero, mapMax,
                    fuMargin, rpMargin, fu.pheromone,
                    fuSecond == null ? Double.NaN : fuSecond.pheromone,
                    fuSecond == null ? -1 : fuSecond.r, fuSecond == null ? -1 : fuSecond.c,
                    rpSecond == null ? -1 : rpSecond.r, rpSecond == null ? -1 : rpSecond.c);
        }
        LinkedList<Point> path = new LinkedList<>();
        Node cursor = new Node(chosen.r, chosen.c, chosen.distance, chosen.distance,
                parent[chosen.r][chosen.c]);
        while (cursor != null) {
            path.addFirst(new Point(cursor.r, cursor.c));
            cursor = cursor.parent;
        }
        return path;
    }

    /** Optimistic IG-2: 360 rays, 1 degree, 0.125 cell; known obstacles stop rays. */
    private int expectedSensingGain(int id, int r, int c) {
        LocalDroneMap m = localMaps[id];
        BitSet unknown = new BitSet(rows * cols);
        double maxRadius = drones.get(id).sensorFoV + 0.5; // same legacy sensor bound
        for (int degree = 0; degree < 360; degree++) {
            double angle = Math.toRadians(degree);
            double dr = Math.sin(angle), dc = Math.cos(angle);
            int previous = -1;
            for (double distance = 0.125; distance <= maxRadius; distance += 0.125) {
                int rr = (int) Math.floor(r + 0.5 + dr * distance);
                int cc = (int) Math.floor(c + 0.5 + dc * distance);
                if (!inside(rr, cc)) break;
                int index = rr * cols + cc;
                if (index == previous) continue;
                previous = index;
                int occupancy = m.get(rr, cc);
                if (occupancy == LocalDroneMap.OCCUPIED) break;
                if (occupancy == LocalDroneMap.UNKNOWN) unknown.set(index);
            }
        }
        return unknown.cardinality();
    }

    /** Mean pheromone in 3x3, counting known FREE cells only. */
    private double meanFrontierPheromone(int id, int r, int c) {
        double sum = 0;
        int count = 0;
        for (int dr = -1; dr <= 1; dr++)
            for (int dc = -1; dc <= 1; dc++) {
                int rr = r + dr, cc = c + dc;
                if (!inside(rr, cc) || localMaps[id].get(rr, cc) != LocalDroneMap.FREE) continue;
                sum += localPheromones[id].get(rr, cc);
                count++;
            }
        return count == 0 ? 0 : sum / count;
    }

    /** The same scoring equation is used by FU (lambda=0) and Proposed. */
    static double frontierUtility(double gain, double distance, double pheromone, double lambda) {
        return gain - distance - lambda * pheromone;
    }

    /** Deterministic diagnostics for focused tests; no ground truth access. */
    public int getExpectedGain(int droneId, int r, int c) {
        return expectedSensingGain(droneId, r, c);
    }
    public double getFrontierPheromone(int droneId, int r, int c) {
        return meanFrontierPheromone(droneId, r, c);
    }
    public String getFrontierMethod() { return FRONTIER_METHOD; }
    public double getPheromoneWeight() { return pheromoneWeight; }
    private record FrontierScore(int r, int c, double gain, double distance, double pheromone) { }
    private int currentDiagnosticStep = -1;
    private boolean betterDiagnostic(FrontierScore candidate, double utility, FrontierScore current, double currentUtility) {
        return current == null || utility > currentUtility + 1e-9 ||
                (Math.abs(utility - currentUtility) <= 1e-9 &&
                    (candidate.distance < current.distance - 1e-9 ||
                     (Math.abs(candidate.distance - current.distance) <= 1e-9 &&
                      candidate.r * cols + candidate.c < current.r * cols + current.c)));
    }

    private void simulate() {
        int stagnant = 0;
        int previousObserved = observedReachable;
        for (int t = 0; t < MAX_STEPS; t++) {
            currentDiagnosticStep = t;
            for (int i = 0; i < drones.size(); i++) {
                sense(i, row[i], col[i], drones.get(i).sensorFoV, t);
            }
            // Stage 5: evaporate before sharing; no deposit for hovering.
            // Legacy mode is completely unchanged for numeric regression.
            if (LOCAL_NAVIGATION)
                for (LocalPheromoneMap field : localPheromones) field.evaporate(evaporationRate);
            // Stage 4: occupancy exchange; Stage 5: pheromone max exchange.
            // Both read snapshots from BEFORE their respective exchanges.
            // Disabled in legacy mode to preserve original regression behavior.
            double[][][] pheromoneBefore = null;
            if (LOCAL_NAVIGATION && "pheromone".equals(FRONTIER_METHOD)) {
                pheromoneBefore = new double[drones.size()][][];
                for (int i = 0; i < drones.size(); i++) pheromoneBefore[i] = localPheromones[i].snapshot();
            }
            if (LOCAL_NAVIGATION) {
                receivedCellUpdates += LocalMapExchange.exchange(
                        localMaps, row, col, communicationRadius, localFrontiers);
                sharedPheromoneUpdates += LocalPheromoneExchange.exchange(
                        localPheromones, row, col, communicationRadius);
            }
            boolean[] occupancyEvent = new boolean[drones.size()];
            boolean[] peerPheromoneEvent = new boolean[drones.size()];
            for (int i = 0; i < drones.size(); i++) {
                occupancyEvent[i] = localMaps[i].knownCount() != lastKnownCount[i];
                // Occupancy changes constitute a replanning event. Pheromone
                // received via communication is handled below in proposed mode.
                if (localMaps[i].knownCount() != lastKnownCount[i]) needsSelection[i] = true;
                lastKnownCount[i] = localMaps[i].knownCount();
                localFrontiers[i].update(localMaps[i]);
                if (VERIFY_FTFD && !localFrontiers[i].snapshot().equals(IncrementalFrontierSet.fullScan(localMaps[i])))
                    throw new IllegalStateException("FTFD mismatch drone=" + i + " timestep=" + t);
            }
            // Only changes in a drone's own frontier neighborhood trigger
            // pheromone-based reselection; remote/unrelated field updates do not.
            if (pheromoneBefore != null) for (int i = 0; i < drones.size(); i++) {
                BitSet frontiers = localFrontiers[i].snapshot();
                outer: for (int key = frontiers.nextSetBit(0); key >= 0; key = frontiers.nextSetBit(key+1)) {
                    int fr = key / cols, fc = key % cols;
                    for (int dr = -1; dr <= 1; dr++) for (int dc = -1; dc <= 1; dc++) {
                        int rr = fr + dr, cc = fc + dc;
                        if (inside(rr,cc) && localMaps[i].get(rr,cc)==LocalDroneMap.FREE &&
                                localPheromones[i].get(rr,cc) > pheromoneBefore[i][rr][cc] + 1e-12) {
                            needsSelection[i] = true;
                            peerPheromoneEvent[i] = true;
                            break outer;
                        }
                    }
                }
            }
            if (observedReachable == reachableFree) {
                terminationReason = LOCAL_NAVIGATION ? "EXPLORATION_COMPLETE" : "ALL_REACHABLE_FREE_CELLS_OBSERVED";
                steps = t;
                break;
            }
            if (!LOCAL_NAVIGATION) detectOwnedFrontiers();
            int n = drones.size();
            Point[] next = new Point[n];
            for (int i = 0; i < n; i++) {
                Point goal = goals[i];
                boolean valid = goal != null && (LOCAL_NAVIGATION
                        ? localFrontier(i, goal.row(), goal.col())
                        : owner[goal.row()][goal.col()] == i && isFrontier(goal.row(), goal.col()));
                boolean scoring = LOCAL_NAVIGATION && !"nearest".equals(FRONTIER_METHOD);
                boolean waitingWithoutEvent = LOCAL_NAVIGATION && goal == null &&
                        routes[i].isEmpty() && !needsSelection[i];
                if (!waitingWithoutEvent && (!valid || routes[i].size() < 2 ||
                        (LOCAL_NAVIGATION && needsSelection[i]))) {
                    if (LOCAL_NAVIGATION) {
                        targetReselections++;
                        if (diagnostics.enabled()) diagnostics.replan(t, i, FRONTIER_METHOD,
                                occupancyEvent[i], peerPheromoneEvent[i], !valid,
                                routes[i].size() < 2, goals[i]);
                    }
                    List<Point> fresh = !LOCAL_NAVIGATION ? astarNearest(i) :
                            (scoring ? scoredLocalFrontier(i) : localShortestFrontier(i));
                    needsSelection[i] = false;
                    routes[i] = fresh;
                    goals[i] = fresh.isEmpty() ? null : fresh.get(fresh.size() - 1);
                }
                // If the next waypoint was invalidated, replan again.
                if (routes[i].size() > 1) {
                    Point step = routes[i].get(1);
                    next[i] = (LOCAL_NAVIGATION
                            ? localLegal(i, row[i], col[i], step.row(), step.col())
                            : legal(row[i], col[i], step.row(), step.col()))
                            ? step : new Point(row[i], col[i]);
                } else next[i] = new Point(row[i], col[i]);
            }
            if (LOCAL_NAVIGATION) {
                Point[] current = new Point[n];
                for (int i = 0; i < n; i++) current[i] = new Point(row[i], col[i]);
                Point[] safe = RotatingCollisionResolver.resolve(current, next, t);
                for (int i = 0; i < n; i++) {
                    if ((next[i].row() != row[i] || next[i].col() != col[i]) &&
                            safe[i].row() == row[i] && safe[i].col() == col[i]) blockedMoves++;
                }
                next = safe;
            } else {
            // Simultaneous collision filtering, lowest ID wins destination tie.
            for (int i = 0; i < n; i++)
                for (int j = i + 1; j < n; j++) {
                    if (next[i].row() == next[j].row() && next[i].col() == next[j].col())
                        next[j] = new Point(row[j], col[j]);
                }
            for (int k = 0; k < n * n; k++) {
                boolean changed = false;
                for (int i = 0; i < n; i++)
                    for (int j = 0; j < n; j++)
                        if (i != j) {
                            if (next[i].row() == row[j] && next[i].col() == col[j] && next[j].row() == row[j] && next[j].col() == col[j]) {
                                next[i] = new Point(row[i], col[i]);
                                changed = true;
                            }
                            if (i < j && next[i].row() == row[j] && next[i].col() == col[j] && next[j].row() == row[i] && next[j].col() == col[i]) {
                                next[i] = new Point(row[i], col[i]);
                                next[j] = new Point(row[j], col[j]);
                                changed = true;
                            }
                            // Also reject opposite diagonal segments crossing at cell corners.
                            if (i < j && row[i] + row[j] == next[i].row() + next[j].row()
                                    && col[i] + col[j] == next[i].col() + next[j].col()
                                    && (row[i] != next[i].row() || col[i] != next[i].col())
                                    && (row[j] != next[j].row() || col[j] != next[j].col())
                                    && !(row[i] == row[j] && col[i] == col[j])
                                    && (row[i] + next[i].row() == row[j] + next[j].row())
                                    && (col[i] + next[i].col() == col[j] + next[j].col())) {
                                next[j] = new Point(row[j], col[j]);
                                changed = true;
                            }
                        }
                if (!changed) break;
            }
            }
            boolean moved = false;
            for (int i = 0; i < n; i++) {
                int nr = next[i].row(), nc = next[i].col();
                if (nr != row[i] || nc != col[i]) {
                    row[i] = nr;
                    col[i] = nc;
                    visit(i, nr, nc);
                    if (LOCAL_NAVIGATION) localPheromones[i].deposit(nr, nc, depositAmount);
                    moved = true;
                    if (routes[i].size() > 1 && routes[i].get(1).row() == nr && routes[i].get(1).col() == nc)
                        routes[i].remove(0);
                } else if (routes[i].size() > 1) routes[i] = Collections.emptyList();
                drones.get(i).path.add(new Point(row[i], col[i]));
            }
            steps = t + 1;
            boolean progressed = moved || observedReachable > previousObserved;
            previousObserved = observedReachable;
            stagnant = LOCAL_NAVIGATION ? (progressed ? 0 : stagnant + 1) : (moved ? 0 : stagnant + 1);
            if (stagnant >= (LOCAL_NAVIGATION ? stallLimit : 30)) {
                terminationReason = LOCAL_NAVIGATION ? "STALLED" : "STALL_30_STEPS";
                break;
            }
        }
        for (int i = 0; i < drones.size(); i++) {
            sense(i, row[i], col[i], drones.get(i).sensorFoV, steps);
            localFrontiers[i].update(localMaps[i]);
            if (VERIFY_FTFD && !localFrontiers[i].snapshot().equals(IncrementalFrontierSet.fullScan(localMaps[i])))
                throw new IllegalStateException("FTFD mismatch final drone=" + i);
        }
        if (observedReachable == reachableFree) terminationReason = LOCAL_NAVIGATION ? "EXPLORATION_COMPLETE" : "ALL_REACHABLE_FREE_CELLS_OBSERVED";
        if (terminationReason.equals("NOT_STARTED")) terminationReason = LOCAL_NAVIGATION ? "TIMEOUT" : "MAX_STEPS";
        for (Drone d : drones) {
            d.frameIndex = 0;
        }
    }

    /** Returns a defensive snapshot of one drone's physical observations. */
    public int[][] getLocalMapSnapshot(int droneId) {
        return localMaps[droneId].snapshot();
    }

    /** Defensive snapshot of pheromone known to a single drone (never globally aggregated). */
    public double[][] getLocalPheromoneSnapshot(int droneId) {
        return localPheromones[droneId].snapshot();
    }

    public double getEvaporationRate() { return evaporationRate; }
    public double getDepositAmount() { return depositAmount; }
    public int getSharedPheromoneUpdates() { return sharedPheromoneUpdates; }
    public double getCommunicationRadius() { return communicationRadius; }

    public int getReceivedCellUpdates() { return receivedCellUpdates; }

    public String getNavigationMode() {
        return LOCAL_NAVIGATION ? "LOCAL_MAP" : "LEGACY_ORACLE";
    }

    public int getLocalKnownCellCount(int droneId) {
        return localMaps[droneId].knownCount();
    }

    public int[][] getFirstObservedAt() {
        return firstObservedAt;
    }

    public boolean[][] getMustExploreMask() {
        return reachable;
    }

    public int getSteps() {
        return steps;
    }

    public int getBlockedMoves() { return blockedMoves; }
    public int getTargetReselections() { return targetReselections; }
    public double getCoverage() {
        return coverage;
    }

    public double getRevisit() {
        return revisit;
    }

    public double getOverlap() {
        return overlap;
    }

    public double getObservedRatio() {
        return reachableFree == 0 ? 100 : 100.0 * observedReachable / reachableFree;
    }

    public String getTerminationReason() {
        return terminationReason;
    }

    private record Node(int r, int c, double g, double f, Node parent) {
    }
}
