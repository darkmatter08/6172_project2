#include "./Quadtree.h"

// Make new Quadtree, lines and points not set.
Quadtree make_quadtree(unsigned int capacity, double x_lo, double y_lo, 
	double x_hi, double y_hi) {
	Quadtree new_tree = {
		.quadrant_1 = NULL, .quadrant_2 = NULL, .quadrant_3 = NULL,
		.quadrant_4 = NULL, .lines = malloc(sizeof(Line *) * capacity), // TODO:FREE
		.numOfLines = 0, .capacity = capacity, .p1 = { .x = x_lo, .y = y_lo },
		.p2 = { .x = x_hi, .y = y_hi }
	};
	return new_tree;
}

// NOTE: Two strategies for this method: 
// 1) Parse lines from world one-by-one
// 2) Take all lines from world, fix the data structure until it works. 
// Parses the CollisionWorld into a Quadtree
Quadtree* parse_CollisionWorld_to_Quadtree(CollisionWorld * world) {
	Quadtree *tree = malloc(sizeof(Quadtree));
	*tree = make_quadtree(N, BOX_XMIN, BOX_YMIN, BOX_XMAX, BOX_YMAX);
	for (int i = 0; i < world->numOfLines; i++) {
		Line *l = world->lines[i];
		insert_line(l, tree);
	}
	return tree;
}

// Insert into the subtree hanging from (and including) tree
void insert_line(Line* l, Quadtree * tree) {
	if (tree->numOfLines < N && tree->quadrant_1 == NULL) { // not-full leaf
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
			tree->p1.y + (tree->p2.y - tree->p1.y)/2
		);
		tree->quadrant_1 = malloc(sizeof(Quadtree));
		*(tree->quadrant_1) = make_quadtree(N,
			tree->p1.x + (tree->p2.x - tree->p1.x)/2,
			tree->p1.y, 
			tree->p2.x,
			tree->p1.y + (tree->p2.y - tree->p1.y)/2
		);
		tree->quadrant_3 = malloc(sizeof(Quadtree));
		*(tree->quadrant_3) = make_quadtree(N,
			tree->p1.x,
			tree->p1.y + (tree->p2.y - tree->p1.y)/2,
			tree->p1.x + (tree->p2.x - tree->p1.x)/2,
			tree->p2.y
		);
		tree->quadrant_4 = malloc(sizeof(Quadtree));
		*(tree->quadrant_4) = make_quadtree(N,
			tree->p1.x + (tree->p2.x - tree->p1.x)/2, 
			tree->p1.y + (tree->p2.y - tree->p1.y)/2,
			tree->p2.x,
			tree->p2.y
		);
		// reassign stuff currently in tree->line into quadrants
		reassign_current_to_quadrants(tree);
	}
	// not leaf case
	// check where the line fits in
	if (can_fit(l, tree->quadrant_1)) { // q1 case
		
		insert_line(l, tree->quadrant_1);
		return;
	} else if (can_fit(l, tree->quadrant_2)) { // q2 case
		insert_line(l, tree->quadrant_2);
	} else if (can_fit(l, tree->quadrant_3)) { // q3 case
		insert_line(l, tree->quadrant_3);
	} else if (can_fit(l, tree->quadrant_4)) { // q4 case
		insert_line(l, tree->quadrant_4);
	} else { // must go into this node 
		// insert_line(l, tree);
		// double node's line capacity if we are at N
		assert(can_fit(l, tree));

		if (tree->numOfLines == tree->capacity) {
			tree->lines = realloc(tree->lines, sizeof(Line *) * tree->capacity * 2);
			assert(tree->lines);
			tree->capacity *= 2;
		}
		tree->lines[tree->numOfLines++] = l;
		return;
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
	// tree->lines = {0};
	// memset(tree->lines, NULL);
	// assert(tree->numOfLines === 0);
	for (int i = 0; i < current_numOfLines; i++) {
		// Other strategies:
		// 1) pop before insert, shifting
		// 2) New Quadtree into which we insert and reassign pointers for quadtrees
		insert_line(allLines[i], tree);
	}
}

// check if line can fit inside a given Quadtree's boundaries
bool can_fit(Line * line, Quadtree * tree) {
	return 	line->p1.x >= tree->p1.x &&
		line->p1.x < tree->p2.x &&
		line->p2.x >= tree->p1.x &&
		line->p2.x < tree->p2.x &&
		line->p1.y >= tree->p1.y &&
		line->p1.y < tree->p2.y &&
		line->p2.y >= tree->p1.y &&
		line->p2.y < tree->p2.y;
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
	}
	// full leaf
	assert(tree->quadrant_1 == NULL);
	assert(!(tree->quadrant_2));
	assert(!(tree->quadrant_3));
	assert(!(tree->quadrant_4));
	
	free(tree->lines); // leaf case
	free(tree);
}
