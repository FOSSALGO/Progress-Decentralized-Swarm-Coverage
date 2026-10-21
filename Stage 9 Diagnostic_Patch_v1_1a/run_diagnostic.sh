#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ $# -lt 4 ]]; then echo 'Usage: bash tests/run_diagnostic.sh <file.sc> <lambda> <true|false> <output_trace.csv>'; exit 2; fi
OUT="$(mktemp -d)"; trap 'rm -rf "$OUT"' EXIT
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
javac -d "$OUT/classes" "$PKG/Drone.java" Point.java DecentralizedExploration.java tests/ExperimentHarness.java tests/DiagnosticHarness.java
java -ea -cp "$OUT/classes" swarmdrone.decentralizedswarmcoverage_ijrcs.DiagnosticHarness "$1" "$2" "$3" "$4"
