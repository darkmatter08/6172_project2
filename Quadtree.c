#include "./Quadtree.h"

// Make new Quadtree, lines and points not set.
Quadtree make_quadtree(unsigned int capacity, double x_lo, double y_lo, 
  double x_hi, double y_hi, unsigned int depth, Quadtree * parent) {
  Quadtree new_tree = {
    .quadrant_1 = NULL, .quadrant_2 = NULL, .quadrant_3 = NULL,
    .quadrant_4 = NULL, .lines = malloc(sizeof(Line *) * capacity), // TODO:FREE
    .numOfLines = 0, .capacity = capacity, .p1 = { .x = x_lo, .y = y_lo },
    .p2 = { .x = x_hi, .y = y_hi }, .depth=depth, .parent = parent
  };
  return new_tree;
}

// NOTE: Two strategies for this method: 
// 1) Parse lines from world one-by-one
// 2) Take all lines from world, fix the data structure until it works. 
// Parses the CollisionWorld into a Quadtree
Quadtree* parse_CollisionWorld_to_Quadtree(CollisionWorld * world) {
  Quadtree *tree = malloc(sizeof(Quadtree));
  *tree = make_quadtree(N, BOX_XMIN, BOX_YMIN, BOX_XMAX, BOX_YMAX, 0, NULL);
  for (int i = 0; i < world->numOfLines; i++) {
    Line *l = world->lines[i];
    insert_line(l, tree);
  }
  return tree;
}

// Insert into the subtree hanging from (and including) tree
void insert_line(Line* l, Quadtree * tree) {
  // assert(can_fit(l, tree));
  // when parallelogram falls outside of CollisionWorld boundaries this check 
  // fails, when commented we insert line into root of tree instead

  if (tree->numOfLines < N && tree->quadrant_1 == NULL) { // not-full leaf
    tree->lines[tree->numOfLines++] = l;
    return;
  } else if (tree->depth == MAX_DEPTH) { // at MAX_DEPTH
    // No children should be present
    assert(!(tree->quadrant_1));
    assert(!(tree->quadrant_2));
    assert(!(tree->quadrant_3));
    assert(!(tree->quadrant_4));

    // printf("REACHED MAX DEPTH\n");

    // Same as not leaf - spanning line
    if (tree->numOfLines == tree->capacity) {
      tree->lines = realloc(tree->lines, sizeof(Line *) * tree->capacity * 2);
      assert(tree->lines);
      tree->capacity *= 2;
    }
    tree->lines[tree->numOfLines++] = l;
    return;
  } else if (tree->quadrant_1 == NULL) { // full leaf
    assert(!(tree->quadrant_2));
    assert(!(tree->quadrant_3));
    assert(!(tree->quadrant_4));

    tree->quadrant_2 = malloc(sizeof(Quadtree));
    *(tree->quadrant_2) = make_quadtree(N,
      tree->p1.x,
      tree->p1.y, 
      tree->p1.x + (tree->p2.x - tree->p1.x)/2, 
      tree->p1.y + (tree->p2.y - tree->p1.y)/2,
      tree->depth+1,
      tree
    );
    tree->quadrant_1 = malloc(sizeof(Quadtree));
    *(tree->quadrant_1) = make_quadtree(N,
      tree->p1.x + (tree->p2.x - tree->p1.x)/2,
      tree->p1.y, 
      tree->p2.x,
      tree->p1.y + (tree->p2.y - tree->p1.y)/2,
      tree->depth+1,
      tree
    );
    tree->quadrant_3 = malloc(sizeof(Quadtree));
    *(tree->quadrant_3) = make_quadtree(N,
      tree->p1.x,
      tree->p1.y + (tree->p2.y - tree->p1.y)/2,
      tree->p1.x + (tree->p2.x - tree->p1.x)/2,
      tree->p2.y,
      tree->depth+1,
      tree
    );
    tree->quadrant_4 = malloc(sizeof(Quadtree));
    *(tree->quadrant_4) = make_quadtree(N,
      tree->p1.x + (tree->p2.x - tree->p1.x)/2, 
      tree->p1.y + (tree->p2.y - tree->p1.y)/2,
      tree->p2.x,
      tree->p2.y,
      tree->depth+1,
      tree
    );
    // reassign stuff currently in tree->line into quadrants
    reassign_current_to_quadrants(tree);
  }
  // not leaf case
  // check where the line fits in
  if (can_fit(l, tree->quadrant_1)) { // q1 case
    insert_line(l, tree->quadrant_1);
  } else if (can_fit(l, tree->quadrant_2)) { // q2 case
    insert_line(l, tree->quadrant_2);
  } else if (can_fit(l, tree->quadrant_3)) { // q3 case
    insert_line(l, tree->quadrant_3);
  } else if (can_fit(l, tree->quadrant_4)) { // q4 case
    insert_line(l, tree->quadrant_4);
  } else { // must go into this node 
    // double node's line capacity if we are at N
    if (tree->numOfLines == tree->capacity) {
      tree->lines = realloc(tree->lines, sizeof(Line *) * tree->capacity * 2);
      assert(tree->lines);
      tree->capacity *= 2;
    }
    tree->lines[tree->numOfLines++] = l;
  }
}

void reassign_current_to_quadrants(Quadtree * tree) {
  int current_numOfLines = tree->numOfLines;
  Line* allLines[current_numOfLines];
  for (int i = 0; i < tree->numOfLines; i++) {
    // copy lines to stack and delete lines from tree
    allLines[i] = tree->lines[i];
    tree->lines[i] = NULL; // use memset or some other way?
  }
  tree->numOfLines = 0;
  for (int i = 0; i < current_numOfLines; i++) {
    // Other strategies:
    // 1) pop before insert, shifting
    // 2) New Quadtree into which we insert and reassign pointers for quadtrees
    insert_line(allLines[i], tree);
  }
}

// check if line can fit inside a given Quadtree's boundaries
inline bool can_fit(Line * line, Quadtree * tree) {
  return  
    // check line at beginning of time step
    line->p1.x >= tree->p1.x &&
    line->p1.x < tree->p2.x &&
    line->p2.x >= tree->p1.x &&
    line->p2.x < tree->p2.x &&
    line->p1.y >= tree->p1.y &&
    line->p1.y < tree->p2.y &&
    line->p2.y >= tree->p1.y &&
    line->p2.y < tree->p2.y &&
    // check line at end of time step
    line->p1.x + line->velocity.x >= tree->p1.x &&
    line->p1.x + line->velocity.x < tree->p2.x &&
    line->p2.x + line->velocity.x >= tree->p1.x &&
    line->p2.x + line->velocity.x < tree->p2.x &&
    line->p1.y + line->velocity.y >= tree->p1.y &&
    line->p1.y + line->velocity.y < tree->p2.y &&
    line->p2.y + line->velocity.y >= tree->p1.y &&
    line->p2.y + line->velocity.y < tree->p2.y;
}

// Recursively deletes all Quadtrees in this subtree
// on return, the pointer is invalid. 
// This function doesn't successfully free the leaves of the root (1st call)
// so the assert now on line 143 (originally 141) keeps failing. free() doesn't
// actually free the subtree for some reason. 
void delete_Quadtree(Quadtree * tree) {
  if (tree->quadrant_1) { // not leaf
    assert(tree->quadrant_2);
    assert(tree->quadrant_3);
    assert(tree->quadrant_4);

    delete_Quadtree(tree->quadrant_1);
    delete_Quadtree(tree->quadrant_2);
    delete_Quadtree(tree->quadrant_3);
    delete_Quadtree(tree->quadrant_4);
    tree->quadrant_1 = tree->quadrant_2 = tree->quadrant_3 = tree->quadrant_4 = NULL;
  }
  // full leaf
  assert(tree->quadrant_1 == NULL);
  assert(!(tree->quadrant_2));
  assert(!(tree->quadrant_3));
  assert(!(tree->quadrant_4));
  
  free(tree->lines); // leaf case
  free(tree);
}

// Change return type to a struct that is a struct with Line** and pointers to the start of quadtrees (Line **)
void Quadtree_to_Line_array(Quadtree * tree, Line ** lines, int * quadtrees, int* quadtree_numOfLines) {
  assert(tree);
  assert(lines);
  assert(quadtrees);
  assert(quadtree_numOfLines);

  int lines_added = 0;
  int quadtrees_added = 0;

  // BFS
  queue * myq = malloc(sizeof(queue));
  init_queue(myq);

  enqueue(myq, tree);
  // push null to indicate end of level????

  while (!empty(myq)) {
    Quadtree * current = dequeue(myq);

    quadtrees[quadtrees_added] = lines_added;
    quadtree_numOfLines[quadtrees_added] = current->numOfLines;
    quadtrees_added++;

    for (int i = 0; i < current->numOfLines; i++) {
      lines[lines_added++] = current->lines[i]; // replace with memcpy?
    }
    // add null
    // lines[lines_added] = NULL;

    if (current->quadrant_1) { // not leaf
      assert(current->quadrant_2);
      assert(current->quadrant_3);
      assert(current->quadrant_4);

      enqueue(myq, current->quadrant_1);
      enqueue(myq, current->quadrant_2);
      enqueue(myq, current->quadrant_3);
      enqueue(myq, current->quadrant_4);
    }
  }
  free(myq);
}

void detect_collisions(Quadtree * tree, int numOfLines, IntersectionEventList_reducer * X) {
  assert(tree);
  assert(X);

  //// Convert Quadtree to Line Array /////
  Line ** lines = malloc(sizeof(Line*) * numOfLines);

  // invariant -- always construct a full tree
  // overallocate, extra points are 0.
  // quadtrees[i] gives the index into lines[] that corresponds to the first line in that quadtree.
  // i.e. lines[quadtrees[i]] = 1st line in the ith quadtree
  int * quadtrees = calloc(numOfLines, sizeof(int));
  // quadtree_numOfLines[i] is the number of lines in the quadtree at quadtrees[i]
  int * quadtree_numOfLines = calloc(numOfLines, sizeof(int*));

  assert(lines);
  assert(quadtrees);
  assert(quadtree_numOfLines);

  Quadtree_to_Line_array(tree, lines, quadtrees, quadtree_numOfLines);

  //// END Convert Quadtree to Line Array /////

  // Line ** tree = quadtree[i];
  // Line ** tree_next = quadtree[i+1];
  // int numLines = (tree_next - tree) / sizeof((Line *)); // correct?

  detect_collisions_recursive_block(lines, quadtrees, quadtree_numOfLines, 0, 0, X); // Call on root

  // FREE Memory for Convert Quadtree to Line array
  free(lines);
  free(quadtrees);
}

// TODO: Change to accept line array Line**
// Checks lines in quadtree check_source against the lines in the sub-quadtree tree recursively
void detect_collisions_recursive_block(Line ** lines, int * quadtrees, int * quadtree_numOfLines, int tree_index, int tree_depth, IntersectionEventList_reducer * X) {
  // check everything at the root -- walk forward
  cilk_for (int i = quadtrees[tree_index]; i < quadtrees[tree_index] + quadtree_numOfLines[tree_index]; i++) {
    for (int j = i + 1; j < quadtrees[tree_index] + quadtree_numOfLines[tree_index]; j++) {
      Line * l1 = lines[i];
      Line * l2 = lines[j];

      if (compareLines(l1, l2) > 0) {
        Line *temp = l1;
        l1 = l2;
        l2 = temp;
      }

      // Get relative velocity.
      Vec velocity = {.x = l2->velocity.x - l1->velocity.x, .y = l2->velocity.y - l1->velocity.y};

      // Get the parallelogram.
      Vec p1 = {.x = l2->p1.x + velocity.x * 0.5, .y = l2->p1.y + velocity.y * 0.5};
      Vec p2 = {.x = l2->p2.x + velocity.x * 0.5, .y = l2->p2.y + velocity.y * 0.5};

      // l2 parallelogram: p1, p2, l2->p1, l2->p2
      // l2 bounding box:
      vec_dimension l2_tl_x = MIN(MIN(p1.x, p2.x), MIN(l2->p1.x, l2->p2.x));
      vec_dimension l2_tl_y = MIN(MIN(p1.y, p2.y), MIN(l2->p1.y, l2->p2.y));
      vec_dimension l2_br_x = MAX(MAX(p1.x, p2.x), MAX(l2->p1.x, l2->p2.x));
      vec_dimension l2_br_y = MAX(MAX(p1.y, p2.y), MAX(l2->p1.y, l2->p2.y));
      // l1 bounding box:
      vec_dimension l1_tl_x = MIN(l1->p1.x, l1->p2.x);
      vec_dimension l1_tl_y = MIN(l1->p1.y, l1->p2.y);
      vec_dimension l1_br_x = MAX(l1->p1.x, l1->p2.x);
      vec_dimension l1_br_y = MAX(l1->p1.y, l1->p2.y);
      
      // logic:
      if (!( l1_br_x < l2_tl_x || l1_tl_x > l2_br_x || l1_br_y < l2_tl_y || l1_tl_y > l2_br_y )) {
        IntersectionType intersectionType = intersect(l1, l2, 0.5);
        if (intersectionType != NO_INTERSECTION) {
          IntersectionEventList_appendNode(&REDUCER_VIEW(*X), l1, l2, intersectionType);
        }
      }
    }
  }

  // walk up the tree
  // Quadtree * checking = tree->parent;
  // int depth = (tree_index - 1) / 4;
  cilk_for (int i = 0; i < tree_depth; i++) {
    int checking_index = (tree_index - 1) / 4; //parent
    for (int j = 0; j < i; j++) {
      checking_index = (checking_index - 1) / 4; //->parent;
    }
    // Quadtree * checking = 
  // while (checking != NULL) {

    // all pairs
    for (int check_source_index = quadtrees[checking_index]; check_source_index < quadtrees[checking_index] + quadtree_numOfLines[checking_index]/*checking->numOfLines*/; check_source_index++) {
      for (int tree_line_index = quadtrees[tree_index]; tree_line_index < quadtrees[tree_index] + quadtree_numOfLines[tree_index]; tree_line_index++) {
        Line * l1 = lines[tree_line_index];
        Line * l2 = lines[check_source_index];

        if (compareLines(l1, l2) > 0) {
          Line *temp = l1;
          l1 = l2;
          l2 = temp;
        }

        // Get relative velocity.
        Vec velocity = {.x = l2->velocity.x - l1->velocity.x, .y = l2->velocity.y - l1->velocity.y};

        // Get the parallelogram.
        Vec p1 = {.x = l2->p1.x + velocity.x * 0.5, .y = l2->p1.y + velocity.y * 0.5};
        Vec p2 = {.x = l2->p2.x + velocity.x * 0.5, .y = l2->p2.y + velocity.y * 0.5};

        // l2 parallelogram: p1, p2, l2->p1, l2->p2
        // l2 bounding box:
        vec_dimension l2_tl_x = MIN(MIN(p1.x, p2.x), MIN(l2->p1.x, l2->p2.x));
        vec_dimension l2_tl_y = MIN(MIN(p1.y, p2.y), MIN(l2->p1.y, l2->p2.y));
        vec_dimension l2_br_x = MAX(MAX(p1.x, p2.x), MAX(l2->p1.x, l2->p2.x));
        vec_dimension l2_br_y = MAX(MAX(p1.y, p2.y), MAX(l2->p1.y, l2->p2.y));
        // l1 bounding box:
        vec_dimension l1_tl_x = MIN(l1->p1.x, l1->p2.x);
        vec_dimension l1_tl_y = MIN(l1->p1.y, l1->p2.y);
        vec_dimension l1_br_x = MAX(l1->p1.x, l1->p2.x);
        vec_dimension l1_br_y = MAX(l1->p1.y, l1->p2.y);
        
        // logic:
        if (!( l1_br_x < l2_tl_x || l1_tl_x > l2_br_x || l1_br_y < l2_tl_y || l1_tl_y > l2_br_y )) {
          IntersectionType intersectionType = intersect(l1, l2, 0.5);
          if (intersectionType != NO_INTERSECTION) {
            IntersectionEventList_appendNode(&REDUCER_VIEW(*X), l1, l2, intersectionType);
          }
        }
      }
    }
  }

  if (quadtrees[4*tree_index+1]/*tree->quadrant_1*/) {
    assert(quadtrees[4*tree_index+2]);
    assert(quadtrees[4*tree_index+3]);
    assert(quadtrees[4*tree_index+4]);

    cilk_spawn detect_collisions_recursive_block(lines, quadtrees, quadtree_numOfLines, 4*tree_index+1, tree_depth+1, X);
    cilk_spawn detect_collisions_recursive_block(lines, quadtrees, quadtree_numOfLines, 4*tree_index+2, tree_depth+1, X);
    cilk_spawn detect_collisions_recursive_block(lines, quadtrees, quadtree_numOfLines, 4*tree_index+3, tree_depth+1, X);
    detect_collisions_recursive_block(lines, quadtrees, quadtree_numOfLines, 4*tree_index+4, tree_depth+1, X);

    cilk_sync;
  }
}
