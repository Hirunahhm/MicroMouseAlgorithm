#include <stdio.h>
#include <stdbool.h>
#include "API.h"
#include "maze.h"

/* The simulator only answers getStat in recent builds, so asking for it on an
   older one would block forever waiting for a reply. Off by default; turn it
   on to compare runs after a change to the cost model. */
#define REPORT_STATS 0

static void drawStart(void) {
    API_setColor(0, 0, 'G');
    API_setText(0, 0, "Start");
}

static bool mazeSizeOk(void) {
    if (API_mazeWidth() == MAZE_SIZE && API_mazeHeight() == MAZE_SIZE) return true;
    debug_log("Error: maze size does not match MAZE_SIZE");
    return false;
}

#if REPORT_STATS
static void reportStats(void) {
    char line[64];
    snprintf(line, sizeof(line), "total distance: %g", API_getStat("total-distance"));
    debug_log(line);
    snprintf(line, sizeof(line), "total turns: %g", API_getStat("total-turns"));
    debug_log(line);
    snprintf(line, sizeof(line), "best run turns: %g", API_getStat("best-run-turns"));
    debug_log(line);
    snprintf(line, sizeof(line), "score: %g", API_getStat("score"));
    debug_log(line);
}
#endif

int main(void) {
    int total_steps = 0;

    debug_log("Starting flood fill solver...");
    if (!mazeSizeOk()) return 1;

    resetState();
    drawStart();

    while (1) {
        if (API_wasReset()) {
            debug_log("Reset detected!");
            API_ackReset();
            if (!mazeSizeOk()) return 1;
            resetState();
            API_clearAllColor();
            API_clearAllText();
            drawStart();
            total_steps = 0;
            continue;
        }

        if (++total_steps > MAX_TOTAL_STEPS) {
            debug_log("Step budget exhausted, giving up.");
            break;
        }

        bool stale = false;
        if (current_phase != SPEED_TO_GOAL) {
            /* Only a wall the map did not already have can change the route. */
            if (updateWalls()) stale = true;
        }

        Phase previous = current_phase;

        if (current_phase == EXPLORE_TO_GOAL && atGoal()) {
            API_setColor(mouse_x, mouse_y, 'R');
            debug_log("Reached the goal, heading back to the start.");
            current_phase = EXPLORE_TO_START;

        } else if (current_phase == EXPLORE_TO_START && atStart()) {
            /* Two runs only: one lap out, one lap back, then the speed run.
               Both laps sense every cell they enter, so the route they walked
               is fully known and the pessimistic map always has a way home.
               The cost is that the speed run is the best route through what
               was seen, not the best route in the maze. */
            API_setColor(mouse_x, mouse_y, 'G');
            debug_log("Exploration done, running the best known route.");
            current_phase = SPEED_TO_GOAL;

        } else if (current_phase == SPEED_TO_GOAL && atGoal()) {
            API_setColor(mouse_x, mouse_y, 'R');
            debug_log("Speed run complete.");
            break;
        }

        if (current_phase != previous) {
            stale = true;
            if (current_phase == SPEED_TO_GOAL) {
                computeCostMap(current_phase);
                stale = false;
                showCosts();
            }
        }
        if (stale) computeCostMap(current_phase);

        TurnCmd cmd = nextMove();
        if (cmd == TURN_BLOCKED) {
            debug_log("No move leads closer to the target, giving up.");
            break;
        }

        switch (cmd) {
            case TURN_LEFT:
                API_turnLeft();
                mouse_dir = (Direction)((mouse_dir + 3) & 3);
                break;
            case TURN_RIGHT:
                API_turnRight();
                mouse_dir = (Direction)((mouse_dir + 1) & 3);
                break;
            case TURN_BACK:
                API_turnLeft();
                API_turnLeft();
                mouse_dir = (Direction)((mouse_dir + 2) & 3);
                break;
            default:
                break;
        }

        if (API_moveForward() == 0) {
            debug_log("Crash detected!");
            break;
        }
        applyMove();

        API_setColor(mouse_x, mouse_y,
                     current_phase == SPEED_TO_GOAL ? 'Y' : 'B');
    }

#if REPORT_STATS
    reportStats();
#endif
    debug_log("Run complete.");
    return 0;
}
