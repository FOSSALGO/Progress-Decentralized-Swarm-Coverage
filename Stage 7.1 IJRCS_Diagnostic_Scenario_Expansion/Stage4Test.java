package swarmdrone.decentralizedswarmcoverage_ijrcs;

import java.util.*;

/** Deterministic Stage 4 protocol tests, independent of simulator ground truth. */
public final class Stage4Test {
    private static void check(boolean ok, String message) {
        if (!ok) throw new AssertionError(message);
    }
    private static void updateAndCheck(LocalDroneMap[] maps, IncrementalFrontierSet[] fs) {
        for (int i = 0; i < maps.length; i++) {
            fs[i].update(maps[i]);
            check(fs[i].snapshot().equals(IncrementalFrontierSet.fullScan(maps[i])),
                    "FTFD vs full scan for drone " + i);
        }
    }
    public static void main(String[] args) {
        LocalDroneMap[] m = new LocalDroneMap[3];
        IncrementalFrontierSet[] f = new IncrementalFrontierSet[3];
        for (int i = 0; i < 3; i++) {
            m[i] = new LocalDroneMap(7, 9);
            f[i] = new IncrementalFrontierSet(7, 9);
        }
        // Drone A--B--C form a chain, but A and C are out of communication range.
        int[] row = {3, 3, 3}, col = {1, 3, 5};
        m[2].observe(1, 6, LocalDroneMap.OCCUPIED);
        f[2].changed(1, 6);
        m[0].observe(5, 1, LocalDroneMap.FREE);
        f[0].changed(5, 1);
        updateAndCheck(m, f);
        check(LocalMapExchange.exchange(m, row, col, 2, f) == 2,
                "only A->B and C->B single-hop updates");
        updateAndCheck(m, f);
        check(m[1].get(1, 6) == LocalDroneMap.OCCUPIED, "B received from C");
        check(m[1].get(5, 1) == LocalDroneMap.FREE, "B received from A");
        check(m[0].get(1, 6) == LocalDroneMap.UNKNOWN, "A cannot receive C via B immediately");
        check(m[2].get(5, 1) == LocalDroneMap.UNKNOWN, "C cannot receive A via B immediately");
        check(LocalMapExchange.exchange(m, row, col, 2, f) == 2, "second hop on next tick");
        updateAndCheck(m, f);
        check(m[0].get(1, 6) == LocalDroneMap.OCCUPIED, "A learns C on next tick");
        check(m[2].get(5, 1) == LocalDroneMap.FREE, "C learns A on next tick");
        check(LocalMapExchange.exchange(m, row, col, 2, f) == 0, "repeated updates idempotent");
        updateAndCheck(m, f);
        // No connection means no transfer even if a source knows cells.
        LocalDroneMap[] isolated = {new LocalDroneMap(4, 4), new LocalDroneMap(4, 4)};
        IncrementalFrontierSet[] isolatedF = {new IncrementalFrontierSet(4, 4), new IncrementalFrontierSet(4, 4)};
        isolated[0].observe(2, 2, LocalDroneMap.OCCUPIED);
        isolatedF[0].changed(2, 2);
        check(LocalMapExchange.exchange(isolated, new int[]{0, 3}, new int[]{0, 3}, 2, isolatedF) == 0,
                "out-of-range sends nothing");
        check(isolated[1].knownCount() == 0, "no leakage out of range");
        updateAndCheck(isolated, isolatedF);
        // UNKNOWN->FREE event communicated should create frontier, and further
        // communication should retire an invalid frontier when neighbors become known.
        LocalDroneMap[] frontierMaps = {new LocalDroneMap(3, 3), new LocalDroneMap(3, 3)};
        IncrementalFrontierSet[] frontierSets = {new IncrementalFrontierSet(3, 3), new IncrementalFrontierSet(3, 3)};
        frontierMaps[0].observe(1, 1, LocalDroneMap.FREE);
        frontierSets[0].changed(1, 1);
        check(LocalMapExchange.exchange(frontierMaps, new int[]{1, 1}, new int[]{0, 2}, 2, frontierSets) == 1,
                "new center cell communicated");
        updateAndCheck(frontierMaps, frontierSets);
        check(frontierSets[1].contains(1, 1), "new frontier created after communication");
        for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++)
            if (!(r == 1 && c == 1)) {
                frontierMaps[0].observe(r, c, LocalDroneMap.FREE);
                frontierSets[0].changed(r, c);
            }
        check(LocalMapExchange.exchange(frontierMaps, new int[]{1, 1}, new int[]{0, 2}, 2, frontierSets) == 8,
                "remaining neighbors communicated");
        updateAndCheck(frontierMaps, frontierSets);
        check(!frontierSets[1].contains(1, 1), "frontier invalidation after communication");
        // Snapshot aliasing is impossible: changing a snapshot cannot alter map state.
        int[][] external = m[0].snapshot();
        external[1][6] = LocalDroneMap.UNKNOWN;
        check(m[0].get(1, 6) == LocalDroneMap.OCCUPIED, "snapshot defensive copy");
        System.out.println("PASS Stage 4: one hop, next-tick propagation, range limit, idempotency, FTFD updates, and map isolation");
    }
}
