#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT
PKG="$OUT/stub/swarmdrone/decentralizedswarmcoverage_ijrcs"
mkdir -p "$PKG" "$OUT/classes"
cat > "$PKG/Drone.java" <<'JAVA'
package swarmdrone.decentralizedswarmcoverage_ijrcs;
import java.util.ArrayList;
public class Drone {
 public int initialRowPosition, initialColPosition, sensorFoV, frameIndex;
 public ArrayList<Point> path=new ArrayList<>();
 public Drone(int[][] env,int r,int c,int radius,int cellSize){initialRowPosition=r;initialColPosition=c;sensorFoV=radius;}
}
JAVA
javac -d "$OUT/classes" "$PKG/Drone.java" Point.java DecentralizedExploration.java tests/ExperimentHarness.java
java -ea -cp "$OUT/classes" swarmdrone.decentralizedswarmcoverage_ijrcs.ExperimentHarness "$PWD/environments" "$PWD/tests/results.csv"
