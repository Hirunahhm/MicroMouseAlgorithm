#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "API.h"
#include "maze.h"

/* ---------------------------------------------------------------- state --- */

int       mouse_x, mouse_y;
Direction mouse_dir;
Phase     current_phase;

/* A wall bit is only ever set together with its known bit, so "wall" implies
   "known". An edge that is not known reads as open to the optimistic search
   and as closed to the pessimistic one. */
static bool v_wall[MAZE_SIZE][MAZE_SIZE + 1];
static bool h_wall[MAZE_SIZE + 1][MAZE_SIZE];
static bool v_known[MAZE_SIZE][MAZE_SIZE + 1];
static bool h_known[MAZE_SIZE + 1][MAZE_SIZE];

/* Only used to break ties between equally good moves while exploring. */
static bool visited[MAZE_SIZE][MAZE_SIZE];

/* cost_map[y][x][h] is the cost of the best route to the current target from
   cell (y,x) facing h. Because it is indexed by pose rather than position, it
   only goes stale when a wall is discovered or the target changes, never when
   the mouse moves. cost_mode is the interpretation it was built with, which
   nextMove has to agree with. */
static uint16_t cost_map[MAZE_SIZE][MAZE_SIZE][4];
static uint16_t scratch[MAZE_SIZE][MAZE_SIZE][4];
static WallMode cost_mode = WALLS_OPTIMISTIC;

static const int  DX[4]       = {  0,   1,   0,  -1  };
static const int  DY[4]       = {  1,   0,  -1,   0  };
static const char DIR_CHAR[4] = { 'n', 'e', 's', 'w' };

#define NSTATES        (MAZE_SIZE * MAZE_SIZE * 4)
#define STATE(y, x, h) ((((y) * MAZE_SIZE + (x)) * 4) + (h))

void debug_log(const char *text) {
    fprintf(stderr, "%s\n", text);
    fflush(stderr);
}

/* ------------------------------------------------------------ wall model --- */

static bool *wallPtr(int y, int x, int d) {
    switch (d) {
        case NORTH: return &h_wall[y + 1][x];
        case SOUTH: return &h_wall[y][x];
        case EAST:  return &v_wall[y][x + 1];
        default:    return &v_wall[y][x];
    }
}

static bool *knownPtr(int y, int x, int d) {
    switch (d) {
        case NORTH: return &h_known[y + 1][x];
        case SOUTH: return &h_known[y][x];
        case EAST:  return &v_known[y][x + 1];
        default:    return &v_known[y][x];
    }
}

static bool inBounds(int y, int x) {
    return y >= 0 && y < MAZE_SIZE && x >= 0 && x < MAZE_SIZE;
}

static bool blocked(int y, int x, int d, WallMode mode) {
    if (!inBounds(y + DY[d], x + DX[d])) return true;
    if (*wallPtr(y, x, d)) return true;
    if (mode == WALLS_PESSIMISTIC && !*knownPtr(y, x, d)) return true;
    return false;
}

static bool recordEdge(int y, int x, int d, bool wall) {
    bool *w = wallPtr(y, x, d);
    bool *k = knownPtr(y, x, d);
    bool newly_walled = wall && !*w;
    if (wall) *w = true;
    *k = true;
    return newly_walled;
}

static uint16_t turnCost(int from, int to) {
    int diff = (to - from) & 3;
    if (diff == 0) return 0;
    return (diff == 2) ? (2 * TURN_COST) : TURN_COST;
}

/* ------------------------------------------------------------- odometry --- */

bool updateWalls(void) {
    int  dirs[3]   = { mouse_dir, (mouse_dir + 3) & 3, (mouse_dir + 1) & 3 };
    bool sensed[3];
    bool changed = false;

    sensed[0] = API_wallFront();
    sensed[1] = API_wallLeft();
    sensed[2] = API_wallRight();

    for (int i = 0; i < 3; i++) {
        if (recordEdge(mouse_y, mouse_x, dirs[i], sensed[i])) changed = true;
        if (sensed[i]) API_setWall(mouse_x, mouse_y, DIR_CHAR[dirs[i]]);
    }
    return changed;
}

void applyMove(void) {
    /* The edge just crossed is open, and now known to be. */
    recordEdge(mouse_y, mouse_x, mouse_dir, false);
    mouse_y += DY[mouse_dir];
    mouse_x += DX[mouse_dir];
    visited[mouse_y][mouse_x] = true;
}

bool atGoal(void) {
    return mouse_x >= GOAL_LO && mouse_x <= GOAL_HI &&
           mouse_y >= GOAL_LO && mouse_y <= GOAL_HI;
}

bool atStart(void) {
    return mouse_x == 0 && mouse_y == 0;
}

/* ---------------------------------------------------------------- search --- */

/* Label-correcting shortest path over the 1024 (cell, heading) poses, run
   backwards from the target so that out[y][x][h] is the cost of finishing the
   route from that pose. Costs are unequal, so plain breadth-first search would
   be wrong here; the in-queue flag keeps each pose in the queue at most once,
   which bounds the queue at NSTATES without a heap or a bucket pool. */
static uint16_t q_buf[NSTATES];
static bool     q_in[NSTATES];

static void computeCosts(bool to_goal, WallMode mode,
                         uint16_t out[MAZE_SIZE][MAZE_SIZE][4]) {
    int head = 0, tail = 0, count = 0;

    for (int y = 0; y < MAZE_SIZE; y++)
        for (int x = 0; x < MAZE_SIZE; x++)
            for (int h = 0; h < 4; h++) {
                out[y][x][h] = COST_INF;
                q_in[STATE(y, x, h)] = false;
            }

    if (to_goal) {
        for (int y = GOAL_LO; y <= GOAL_HI; y++)
            for (int x = GOAL_LO; x <= GOAL_HI; x++)
                for (int h = 0; h < 4; h++) {
                    out[y][x][h] = 0;
                    q_buf[tail] = (uint16_t)STATE(y, x, h);
                    q_in[STATE(y, x, h)] = true;
                    tail = (tail + 1) % NSTATES;
                    count++;
                }
    } else {
        for (int h = 0; h < 4; h++) {
            out[0][0][h] = 0;
            q_buf[tail] = (uint16_t)STATE(0, 0, h);
            q_in[STATE(0, 0, h)] = true;
            tail = (tail + 1) % NSTATES;
            count++;
        }
    }

    while (count > 0) {
        uint16_t s = q_buf[head];
        head = (head + 1) % NSTATES;
        count--;
        q_in[s] = false;

        int h = s & 3;
        int x = (s >> 2) % MAZE_SIZE;
        int y = (s >> 2) / MAZE_SIZE;
        uint16_t c = out[y][x][h];

        /* Predecessors: the pose that reaches (y,x,h) came from the cell one
           step back along h, after turning onto h from some heading ph. */
        int py = y - DY[h];
        int px = x - DX[h];
        if (!inBounds(py, px)) continue;
        if (blocked(py, px, h, mode)) continue;

        for (int ph = 0; ph < 4; ph++) {
            uint32_t nc = (uint32_t)c + STEP_COST + turnCost(ph, h);
            if (nc < out[py][px][ph]) {
                out[py][px][ph] = (uint16_t)nc;
                int ns = STATE(py, px, ph);
                if (!q_in[ns]) {
                    q_buf[tail] = (uint16_t)ns;
                    q_in[ns] = true;
                    tail = (tail + 1) % NSTATES;
                    count++;
                }
            }
        }
    }
}

void computeCostMap(Phase phase) {
    cost_mode = (phase == SPEED_TO_GOAL) ? WALLS_PESSIMISTIC : WALLS_OPTIMISTIC;
    computeCosts(phase != EXPLORE_TO_START, cost_mode, cost_map);
}

uint16_t routeCost(WallMode mode) {
    computeCosts(true, mode, scratch);
    return scratch[mouse_y][mouse_x][mouse_dir];
}

bool explorationComplete(void) {
    uint16_t optimistic = routeCost(WALLS_OPTIMISTIC);
    uint16_t pessimistic = routeCost(WALLS_PESSIMISTIC);
    return pessimistic != COST_INF && pessimistic == optimistic;
}

/* Picks the move that lies exactly on a shortest route. The equality test is
   exact because the cost map satisfies the Bellman equation, so a move that
   fails it is not on any optimal route and a pose with no passing move is a
   real error rather than something to paper over. */
TurnCmd nextMove(void) {
    static const TurnCmd candidates[4] = {
        TURN_STRAIGHT, TURN_LEFT, TURN_RIGHT, TURN_BACK
    };
    uint16_t here = cost_map[mouse_y][mouse_x][mouse_dir];
    TurnCmd best = TURN_BLOCKED;
    bool best_is_new = false;
    bool exploring = (current_phase != SPEED_TO_GOAL);

    if (here == COST_INF) return TURN_BLOCKED;

    for (int i = 0; i < 4; i++) {
        int h = (mouse_dir + (int)candidates[i] + 4) & 3;
        if (blocked(mouse_y, mouse_x, h, cost_mode)) continue;

        int ny = mouse_y + DY[h];
        int nx = mouse_x + DX[h];
        uint16_t there = cost_map[ny][nx][h];
        if (there == COST_INF) continue;
        if ((uint32_t)there + STEP_COST + turnCost(mouse_dir, h) != here) continue;

        /* Every survivor costs the same, so preferring an unseen cell while
           exploring is free. */
        bool is_new = !visited[ny][nx];
        if (best == TURN_BLOCKED || (exploring && is_new && !best_is_new)) {
            best = candidates[i];
            best_is_new = is_new;
        }
    }
    return best;
}

/* ----------------------------------------------------------------- misc --- */

void showCosts(void) {
    for (int y = 0; y < MAZE_SIZE; y++) {
        for (int x = 0; x < MAZE_SIZE; x++) {
            uint16_t best = COST_INF;
            for (int h = 0; h < 4; h++)
                if (cost_map[y][x][h] < best) best = cost_map[y][x][h];

            if (best == COST_INF) {
                API_clearText(x, y);
            } else {
                char text[8];
                snprintf(text, sizeof(text), "%u", (unsigned)best);
                API_setText(x, y, text);
            }
        }
    }
}

void resetState(void) {
    for (int y = 0; y < MAZE_SIZE; y++)
        for (int x = 0; x < MAZE_SIZE + 1; x++) {
            v_wall[y][x] = false;
            v_known[y][x] = false;
        }
    for (int y = 0; y < MAZE_SIZE + 1; y++)
        for (int x = 0; x < MAZE_SIZE; x++) {
            h_wall[y][x] = false;
            h_known[y][x] = false;
        }
    for (int y = 0; y < MAZE_SIZE; y++)
        for (int x = 0; x < MAZE_SIZE; x++)
            visited[y][x] = false;

    for (int i = 0; i < MAZE_SIZE; i++) {
        v_wall[i][0]             = v_known[i][0]             = true;
        v_wall[i][MAZE_SIZE]     = v_known[i][MAZE_SIZE]     = true;
        h_wall[0][i]             = h_known[0][i]             = true;
        h_wall[MAZE_SIZE][i]     = h_known[MAZE_SIZE][i]     = true;
    }

    mouse_x = 0;
    mouse_y = 0;
    mouse_dir = NORTH;
    current_phase = EXPLORE_TO_GOAL;
    visited[0][0] = true;
    computeCostMap(EXPLORE_TO_GOAL);
}
