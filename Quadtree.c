#include "./Quadtree.h"

// Make new Quadtree, lines and points not set.
Quadtree make_quadtree(unsigned int capacity, double x_lo, double y_lo,
  double x_hi, double y_hi, unsigned int depth, Quadtree * parent) {
  Quadtree new_tree = {
    .quadrant_1 = NULL, .quadrant_2 = NULL, .quadrant_3 = NULL,
    .quadrant_4 = NULL, .lines = malloc(sizeof(Line *) * capacity),
    .numOfLines = 0, .capacity = capacity, .p1 = { .x = x_lo, .y = y_lo },
    .p2 = { .x = x_hi, .y = y_hi }, .depth = depth, .parent = parent
  };
  return new_tree;
}

// Parse the CollisionWorld into a Quadtree
Quadtree* parse_CollisionWorld_to_Quadtree(CollisionWorld * world, Quadtree ** quadtrees, int * numQuadtrees) {
  Quadtree *tree = malloc(sizeof(Quadtree));
  *tree = make_quadtree(N, BOX_XMIN, BOX_YMIN, BOX_XMAX, BOX_YMAX, 0, NULL);
  quadtrees[*numQuadtrees] = tree;
  *numQuadtrees += 1;
  for (int i = 0; i < world->numOfLines; i++) {
    Line *l = world->lines[i];
    insert_line(l, tree, quadtrees, numQuadtrees);
  }
  return tree;
}

// Insert into the subtree hanging from (and including) tree
void insert_line(Line* l, Quadtree * tree, Quadtree ** quadtrees, int * numQuadtrees) {
  // if leaf node is not full, insert the line
  if (tree->numOfLines < N && tree->quadrant_1 == NULL) {
    tree->lines[tree->numOfLines++] = l;
    return;
  } else if (tree->depth == MAX_DEPTH) {
    assert(tree->quadrant_1 == NULL);
    assert(tree->quadrant_2 == NULL);
    assert(tree->quadrant_3 == NULL);
    assert(tree->quadrant_4 == NULL);

    // if capacity is reached, double it
    if (tree->numOfLines == tree->capacity) {
      tree->lines = realloc(tree->lines, sizeof(Line *) * tree->capacity * 2);
      assert(tree->lines);
      tree->capacity *= 2;
    }
    tree->lines[tree->numOfLines++] = l;
    return;
  } else if (tree->quadrant_1 == NULL) {
    assert(tree->quadrant_2 == NULL);
    assert(tree->quadrant_3 == NULL);
    assert(tree->quadrant_4 == NULL);

    tree->quadrant_1 = malloc(sizeof(Quadtree));
    *(tree->quadrant_1) = make_quadtree(N,
      tree->p1.x + (tree->p2.x - tree->p1.x)/2,
      tree->p1.y,
      tree->p2.x,
      tree->p1.y + (tree->p2.y - tree->p1.y)/2,
      tree->depth+1,
      tree
    );
    tree->quadrant_2 = malloc(sizeof(Quadtree));
    *(tree->quadrant_2) = make_quadtree(N,
      tree->p1.x,
      tree->p1.y,
      tree->p1.x + (tree->p2.x - tree->p1.x)/2,
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

    // reassign stuff currently in tree->line into new quadrants
    reassign_current_to_quadrants(tree, quadtrees, numQuadtrees);

    // add new quadtrees to the array
    quadtrees[*numQuadtrees] = tree->quadrant_1;
    *numQuadtrees += 1;
    quadtrees[*numQuadtrees] = tree->quadrant_2;
    *numQuadtrees += 1;
    quadtrees[*numQuadtrees] = tree->quadrant_3;
    *numQuadtrees += 1;
    quadtrees[*numQuadtrees] = tree->quadrant_4;
    *numQuadtrees += 1;
  }

  // check which quadrant the line fits in, if any
  if (can_fit(l, tree->quadrant_1)) {
    insert_line(l, tree->quadrant_1, quadtrees, numQuadtrees);
  } else if (can_fit(l, tree->quadrant_2)) {
    insert_line(l, tree->quadrant_2, quadtrees, numQuadtrees);
  } else if (can_fit(l, tree->quadrant_3)) {
    insert_line(l, tree->quadrant_3, quadtrees, numQuadtrees);
  } else if (can_fit(l, tree->quadrant_4)) {
    insert_line(l, tree->quadrant_4, quadtrees, numQuadtrees);
  } else {
    // double node's line capacity if full
    if (tree->numOfLines == tree->capacity) {
      tree->lines = realloc(tree->lines, sizeof(Line *) * tree->capacity * 2);
      assert(tree->lines);
      tree->capacity *= 2;
    }
    tree->lines[tree->numOfLines++] = l;
  }
}

void reassign_current_to_quadrants(Quadtree * tree, Quadtree ** quadtrees, int * numQuadtrees) {
  int current_numOfLines = tree->numOfLines;
  Line* allLines[current_numOfLines];
  for (int i = 0; i < tree->numOfLines; i++) {
    allLines[i] = tree->lines[i];
    tree->lines[i] = NULL;
  }
  tree->numOfLines = 0;
  for (int i = 0; i < current_numOfLines; i++) {
    insert_line(allLines[i], tree, quadtrees, numQuadtrees);
  }
}

// Check if line can fit inside a given Quadtree's boundaries
inline bool can_fit(Line * line, Quadtree * tree) {
  return
    // check line at beginning of time step
    line->top_left.x >= tree->p1.x &&
    line->bottom_right.x < tree->p2.x &&
    line->top_left.y >= tree->p1.y &&
    line->bottom_right.y < tree->p2.y &&
    // check line at end of time step
    line->top_left.x + line->velocity.x >= tree->p1.x &&
    line->bottom_right.x + line->velocity.x < tree->p2.x &&
    line->top_left.y + line->velocity.y >= tree->p1.y &&
    line->bottom_right.y + line->velocity.y < tree->p2.y;
}

// Recursively deletes all Quadtrees in this subtree
void delete_Quadtree(Quadtree * tree) {
  if (tree->quadrant_1) {
    assert(tree->quadrant_2);
    assert(tree->quadrant_3);
    assert(tree->quadrant_4);

    // delete child quadtrees
    delete_Quadtree(tree->quadrant_1);
    delete_Quadtree(tree->quadrant_2);
    delete_Quadtree(tree->quadrant_3);
    delete_Quadtree(tree->quadrant_4);
    tree->quadrant_1 = tree->quadrant_2 = tree->quadrant_3 = tree->quadrant_4 = NULL;
  }

  assert(tree->quadrant_1 == NULL);
  assert(tree->quadrant_2 == NULL);
  assert(tree->quadrant_3 == NULL);
  assert(tree->quadrant_4 == NULL);

  // delete itself
  free(tree->lines);
  free(tree);
}

// Check for collisions all quadtrees
void detect_collisions(IntersectionEventList_reducer * reducer, Quadtree ** quadtrees, int * numQuadtrees) {
  cilk_for (int k = 0; k < *numQuadtrees; k++) {
    Quadtree * current_tree = quadtrees[k];

    // check all pairs of lines in the current quadtree node
    cilk_for (int i = 0; i < current_tree->numOfLines; i++) {
      for (int j = i + 1; j < current_tree->numOfLines; j++) {
        Line * l1 = current_tree->lines[i];
        Line * l2 = current_tree->lines[j];

        if (compareLines(l1, l2) > 0) {
          Line *temp = l1;
          l1 = l2;
          l2 = temp;
        }

        IntersectionType intersectionType = intersect(l1, l2);
        if (intersectionType != NO_INTERSECTION) {
          IntersectionEventList_appendNode(&REDUCER_VIEW(*reducer), l1, l2, intersectionType);
        }
      }
    }

    // check all lines in the current quadtree node against all lines in all parent quadtree nodes
    cilk_for (int i = 0; i < current_tree->depth; i++) {
      Quadtree * checking = current_tree->parent;
      for (int j = 0; j < i; j++) {
        checking = checking->parent;
      }

      cilk_for (int check_source_index = 0; check_source_index < checking->numOfLines; check_source_index++) {
        for (int tree_index = 0; tree_index < current_tree->numOfLines; tree_index++) {
          Line * l1 = current_tree->lines[tree_index];
          Line * l2 = checking->lines[check_source_index];

          if (compareLines(l1, l2) > 0) {
            Line *temp = l1;
            l1 = l2;
            l2 = temp;
          }

          IntersectionType intersectionType = intersect(l1, l2);
          if (intersectionType != NO_INTERSECTION) {
            IntersectionEventList_appendNode(&REDUCER_VIEW(*reducer), l1, l2, intersectionType);
          }
        }
      }
    }
  }
}
