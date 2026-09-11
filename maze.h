#ifndef MAZE_H
#define MAZE_H

#include <stdbool.h>
#include <stdint.h>

#define MAZE_SIZE 16

/* Goal region is the 2x2 block at the centre, derived rather than hard coded. */
#define GOAL_LO ((MAZE_SIZE / 2) - 1)
#define GOAL_HI (MAZE_SIZE / 2)

/* Cost model. Entering the next cell costs STEP_COST; every 90 degrees of
   rotation costs TURN_COST on top of that. Raising TURN_COST buys straighter
   routes at the price of travelling more cells. */
#ifndef STEP_COST
#define STEP_COST 1
#endif
#ifndef TURN_COST
#define TURN_COST 2
#endif
#define COST_INF  0xFFFFu

/* Backstops, so a livelock ends with a diagnostic instead of running forever. */
#define MAX_EXPLORE_STEPS 1500
#define MAX_TOTAL_STEPS   3000

typedef enum { NORTH, EAST, SOUTH, WEST } Direction;
typedef enum { EXPLORE_TO_GOAL, EXPLORE_TO_START, SPEED_TO_GOAL } Phase;

/* How an edge that has never been sensed is interpreted. Optimistic drives
   exploration; pessimistic is what the speed run is allowed to rely on. */
typedef enum { WALLS_OPTIMISTIC, WALLS_PESSIMISTIC } WallMode;

typedef enum {
    TURN_LEFT     = -1,
    TURN_STRAIGHT =  0,
    TURN_RIGHT    =  1,
    TURN_BACK     =  2,
    TURN_BLOCKED  =  3
} TurnCmd;

extern int mouse_x, mouse_y;
extern Direction mouse_dir;
extern Phase current_phase;

void debug_log(const char *text);

/* Clears all maze knowledge, puts the mouse back at the start and builds the
   opening cost map. */
void resetState(void);

/* Senses the three walls around the current cell. Returns true only when a
   wall was recorded that the map did not already have, which is the only
   event that can change the optimistic cost map. */
bool updateWalls(void);

/* Call after a successful API_moveForward: records the crossed edge as open
   and advances the tracked position. */
void applyMove(void);

void     computeCostMap(Phase phase);
TurnCmd  nextMove(void);

/* Cost of the best route to the goal from the current pose under the given
   wall interpretation. */
uint16_t routeCost(WallMode mode);

/* True once the optimistic and pessimistic routes agree, which proves that
   no further exploration can shorten the speed run. */
bool explorationComplete(void);

bool atGoal(void);
bool atStart(void);
void showCosts(void);

#endif /* MAZE_H */
