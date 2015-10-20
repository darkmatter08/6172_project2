#include <stdlib.h>
#include "./Line.h"
#include "./Vec.h"
#include "./CollisionWorld.h"
#include "./IntersectionDetection.h"
#include "./IntersectionEventList.h"
#include <assert.h>
#include <stdio.h>
#include "./IntersectionEventListReducer.h"

// N is the maximum number of items in a Quadtree before the 
// quadtree adds its children. 
#define N 64
// Max Quadtree Depth
#define MAX_DEPTH 2

typedef struct Quadtree Quadtree;

struct Quadtree {
  // Space is divided into quadrants when more than N lines are 
  // in the quadrant. 
  // Either all quadrants point to NULL or all point to valid
  // addresses.
  // Quadrants are owned by this Quadtree
  Quadtree * quadrant_1;
  Quadtree * quadrant_2;
  Quadtree * quadrant_3;
  Quadtree * quadrant_4;
  
  // Does not own the lines.
  // Pointer to pointer of lines in this node.
  // lines owned by descendants are not represented here.
  // Lines are owned 
  Line** lines;
  unsigned int numOfLines;
  // add capacity
  unsigned int capacity;

  // add in vectors for region it covers
  Vec p1;  // Lower value corner
  Vec p2;  // Higher value corner

  unsigned int depth;

  // null if root
  Quadtree * parent;
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

void detect_collisions(Quadtree * tree, IntersectionEventList_reducer * X, Quadtree ** quadtrees, int * numQuadtrees);
