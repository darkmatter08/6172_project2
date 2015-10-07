// N is the maximum number of items in a Quadtree before the 
// quadtree adds its children. 
#define N 3

struct Quadtree {
  // Space is divided into quadrants when more than N lines are 
  // in the quadrant. 
  // Either all quadrants point to NULL or all point to valid
  // addresses. 
  Quadtree quadrant_1;
  Quadtree quadrant_2;
  Quadtree quadrant_3;
  Quadtree quadrant_4;
  
  // Does not own the lines.
  // Pointer to pointer of lines in this node.
  // lines owned by descendants are not represented here.
  // Lines are owned 
  Line** lines;
  unsigned int numOfLines;

  // add in vectors for region it covers
  Vec p1;  // Lower value corner
  Vec p2;  // Higher value corner
};
typedef struct Quadtree Quadtree;

// Make new Quadtree
Quadtree make_quadtree();

// Parses the CollisionWorld into a Quadtree
void parse_CollisionWord_to_Quadtree(CollisionWord * world);

// Recursively deletes all Quadtrees in this subtree
void delete_Quadtree(Quadtree * tree);

// Identify collisions
//void detectCollisions(Quadtree * tree);
