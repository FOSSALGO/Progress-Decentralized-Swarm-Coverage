Stage 4 headless tests use tests/Drone.java as a minimal stub ONLY. Never copy this stub into JavaFX GUI sources.
See README_STAGE4.md for javac/java commands. Stage4Test: one-hop exchange, delayed relay, disconnection, repeated exchange, frontier invalidation/creation, and defensive snapshots. Stage3Test: 3,024 randomized frontier comparisons and four end-to-end scenarios with optional per-timestep verification. Stage3 legacy mode regression compared with original Stage 3 byte-for-byte output metrics across four environments.

Stage 5 adds `Stage5Test.java`, which tests bounded local pheromone fields, deposition, evaporation, one-hop MAX merge, delayed relay, idempotence, disconnected pairs, snapshots, and basic simulator integration. Use the Stage 5 README for the compile command.
