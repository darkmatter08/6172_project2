#include <stdlib.h>
#include "./Line.h"
#include "./Vec.h"
#include "./CollisionWorld.h"
#include "./IntersectionDetection.h"
#include "./IntersectionEventList.h"
#include <assert.h>
#include <stdio.h>

// N is the maximum number of items in a Quadtree before the 
// quadtree adds its children. 
#define N 24
// Max Quadtree Depth
#define MAX_DEPTH 24

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
};

// Make new Quadtree
Quadtree make_quadtree(unsigned int capacity, double x_lo, double y_lo, 
  double x_hi, double y_hi, unsigned int depth);

// Parses the CollisionWorld into a Quadtree
Quadtree* parse_CollisionWorld_to_Quadtree(CollisionWorld * world);

// inserts line into Quadtree
void insert_line(Line* l, Quadtree * tree);

// reinserts lines currently in Quadtree back in (called after child Quadtrees are created)
void reassign_current_to_quadrants(Quadtree * tree);

// check if line can fit inside a given Quadtree's boundaries
bool can_fit(Line * line, Quadtree * tree);

// Recursively deletes all Quadtrees in this subtree
void delete_Quadtree(Quadtree * tree);

// Identify collisions recursively for a given line
unsigned int detect_collisions_recursive(Line * line, Quadtree * tree, CollisionWorld * collisionWorld, IntersectionEventList * intersectionEventList);

// Identify collisions in a particular quadtree
unsigned int detect_collisions(Quadtree * tree, CollisionWorld * collisionWorld, IntersectionEventList * intersectionEventList);

// Counts number of lines in the subtree including and hanging from node tree
// int countLinesInSubtree(Quadtree * tree);
