#include "./Quadtree.h"

// Make new Quadtree, lines and points not set.
Quadtree make_quadtree(unsigned int capacity, double x_lo, double y_lo, 
	double x_hi, double y_hi, unsigned int depth) {
	Quadtree new_tree = {
		.quadrant_1 = NULL, .quadrant_2 = NULL, .quadrant_3 = NULL,
		.quadrant_4 = NULL, .lines = malloc(sizeof(Line *) * capacity), // TODO:FREE
		.numOfLines = 0, .capacity = capacity, .p1 = { .x = x_lo, .y = y_lo },
		.p2 = { .x = x_hi, .y = y_hi }, .depth=depth
	};
	return new_tree;
}

// NOTE: Two strategies for this method: 
// 1) Parse lines from world one-by-one
// 2) Take all lines from world, fix the data structure until it works. 
// Parses the CollisionWorld into a Quadtree
Quadtree* parse_CollisionWorld_to_Quadtree(CollisionWorld * world) {
	Quadtree *tree = malloc(sizeof(Quadtree));
	*tree = make_quadtree(N, BOX_XMIN, BOX_YMIN, BOX_XMAX, BOX_YMAX, 0);
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
			tree->depth+1
		);
		tree->quadrant_1 = malloc(sizeof(Quadtree));
		*(tree->quadrant_1) = make_quadtree(N,
			tree->p1.x + (tree->p2.x - tree->p1.x)/2,
			tree->p1.y, 
			tree->p2.x,
			tree->p1.y + (tree->p2.y - tree->p1.y)/2,
			tree->depth+1
		);
		tree->quadrant_3 = malloc(sizeof(Quadtree));
		*(tree->quadrant_3) = make_quadtree(N,
			tree->p1.x,
			tree->p1.y + (tree->p2.y - tree->p1.y)/2,
			tree->p1.x + (tree->p2.x - tree->p1.x)/2,
			tree->p2.y,
			tree->depth+1
		);
		tree->quadrant_4 = malloc(sizeof(Quadtree));
		*(tree->quadrant_4) = make_quadtree(N,
			tree->p1.x + (tree->p2.x - tree->p1.x)/2, 
			tree->p1.y + (tree->p2.y - tree->p1.y)/2,
			tree->p2.x,
			tree->p2.y,
			tree->depth+1
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

// line is not in tree
void detect_collisions_recursive(Line * line, Quadtree * tree, CollisionWorld * collisionWorld, IntersectionEventList_reducer * X) {
	assert(tree);

	// do intersection detection at root of quadtree
	for (int i = 0; i < tree->numOfLines; i++) {
		Line * l1 = tree->lines[i];
		Line * l2 = line;

		if (compareLines(l1, l2) > 0) {
			Line *temp = l1;
			l1 = l2;
			l2 = temp;
		}

		IntersectionType intersectionType = intersect(l1, l2, collisionWorld->timeStep);
		if (intersectionType != NO_INTERSECTION) {
			IntersectionEventList_appendNode(&REDUCER_VIEW(*X), l1, l2, intersectionType);
			// collisionWorld->numLineLineCollisions++;
		}
	}

	// check if children exist
	if (tree->quadrant_1) {
		assert(tree->quadrant_2);
		assert(tree->quadrant_3);
		assert(tree->quadrant_4);

		detect_collisions_recursive(line, tree->quadrant_1, collisionWorld, X);
		detect_collisions_recursive(line, tree->quadrant_2, collisionWorld, X);
		detect_collisions_recursive(line, tree->quadrant_3, collisionWorld, X);
		detect_collisions_recursive(line, tree->quadrant_4, collisionWorld, X);
	}
}

void detect_collisions(Quadtree * tree, CollisionWorld * collisionWorld, IntersectionEventList_reducer * X) {
	assert(tree);

	// IntersectionEventList_reducer X = CILK_C_INIT_REDUCER(
	// 	/* type */ IntersectionEventList, IntersectionEventList_reduce, 
	// 	IntersectionEventList_identity, IntersectionEventList_destroy,
	// 	/* initial value */ (IntersectionEventList) { .head = NULL, .tail = NULL }
	// );

	// CILK_C_REGISTER_REDUCER(X);

	// if children exist, do this recursively
	if (tree->quadrant_1) {
		assert(tree->quadrant_2);
		assert(tree->quadrant_3);
		assert(tree->quadrant_4);

		// cilk_spawn here
		cilk_spawn detect_collisions(tree->quadrant_1, collisionWorld, X);
		cilk_spawn detect_collisions(tree->quadrant_2, collisionWorld, X);
		cilk_spawn detect_collisions(tree->quadrant_3, collisionWorld, X);
		detect_collisions(tree->quadrant_4, collisionWorld, X);

		cilk_sync;
	}

	// check everything at the root
	cilk_for (int i = 0; i < tree->numOfLines; i++) {
		for (int j = i + 1; j < tree->numOfLines; j++) {
			Line * l1 = tree->lines[i];
			Line * l2 = tree->lines[j];

			if (compareLines(l1, l2) > 0) {
				Line *temp = l1;
				l1 = l2;
				l2 = temp;
			}

			IntersectionType intersectionType = intersect(l1, l2, collisionWorld->timeStep);
			if (intersectionType != NO_INTERSECTION) {
				IntersectionEventList_appendNode(&REDUCER_VIEW(*X), l1, l2, intersectionType);
				// collisionWorld->numLineLineCollisions++;
			}
		}

		if (tree->quadrant_1) {
			assert(tree->quadrant_2);
			assert(tree->quadrant_3);
			assert(tree->quadrant_4);

			detect_collisions_recursive(tree->lines[i], tree->quadrant_1, collisionWorld, X);
			detect_collisions_recursive(tree->lines[i], tree->quadrant_2, collisionWorld, X);
			detect_collisions_recursive(tree->lines[i], tree->quadrant_3, collisionWorld, X);
			detect_collisions_recursive(tree->lines[i], tree->quadrant_4, collisionWorld, X);
		}
	}

	// *intersectionEventList_return = X.value;

	// CILK_C_UNREGISTER_REDUCER(X);
}