#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include "./Line.h"
#include "./Vec.h"
#include "./CollisionWorld.h"
#include "./IntersectionDetection.h"
#include "./IntersectionEventList.h"
#include "./IntersectionEventListReducer.h"

// maximum number of items in a Quadtree before the quadtree adds its children
#define N 64
// max Quadtree depth
#define MAX_DEPTH 2

typedef struct Quadtree Quadtree;

struct Quadtree {
  Quadtree * parent;

  Quadtree * quadrant_1;
  Quadtree * quadrant_2;
  Quadtree * quadrant_3;
  Quadtree * quadrant_4;

  Line** lines;
  unsigned int numOfLines;
  unsigned int capacity;
  unsigned int depth;

  // vectors representing quadtree boundaries
  Vec p1;
  Vec p2;
};

// Make new Quadtree
Quadtree make_quadtree(unsigned int capacity, double x_lo, double y_lo,
  double x_hi, double y_hi, unsigned int depth, Quadtree * parent);

// Parses the CollisionWorld into a Quadtree
Quadtree* parse_CollisionWorld_to_Quadtree(CollisionWorld * world, Quadtree ** quadtrees, int * numQuadtrees);

// inserts line into Quadtree
void insert_line(Line* l, Quadtree * tree, Quadtree ** quadtrees, int * numQuadtrees);

// reinserts lines currently in Quadtree back in (called after child Quadtrees are created)
void reassign_current_to_quadrants(Quadtree * tree, Quadtree ** quadtrees, int * numQuadtrees);

// check if line can fit inside a given Quadtree's boundaries
bool can_fit(Line * line, Quadtree * tree);

// Recursively deletes all Quadtrees in this subtree
void delete_Quadtree(Quadtree * tree);

void detect_collisions(IntersectionEventList_reducer * reducer, Quadtree ** quadtrees, int * numQuadtrees);
