#include <"./Quadtree.h">
#include <"./Line.h">

// Make new Quadtree, lines and points not set.
Quadtree make_quadtree(unsigned int capacity, double x_lo, double y_lo, 
	double x_hi, double y_hi) {
	Quadtree new_tree = {
		.quadrant_1 = NULL, .quadrant_2 = NULL, .quadrant_3 = NULL,
		.quadrant_4 = NULL, .line = malloc(sizeof(Line *) * capacity), // TODO:FREE
		.numOfLines = 0, .p1 = { .x = x_lo, .y = y_lo },
		.p2 = { .x = x_hi, .y = y_hi }
	};
	return new_tree;
}

// NOTE: Two strategies for this method: 
// 1) Parse lines from world one-by-one
// 2) Take all lines from world, fix the data structure until it works. 
// Parses the CollisionWorld into a Quadtree
Quadtree* parse_CollisionWord_to_Quadtree(CollisionWorld * world) {
	Quadtree *tree = malloc(sizeof(Quadtree));
	*tree = make_quadtree(N, BOX_XMIN, BOX_YMIN, BOX_XMAX, BOX_YMAX); // need some type of malloc here.
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
		assert(tree->quadrant_2);
		assert(tree->quadrant_3);
		assert(tree->quadrant_4);

		tree->quadrant_2 = malloc(sizeof(Quadtree));
		*(tree->quadrant_2) = make_quadtree(N,
			tree->p1->x,
			tree->p1->y, 
			tree->p1->x + (tree->p2->x - tree->p1->x)/2, 
			tree->p1->y + (tree->p2->y - tree->p1->y)/2
		);
		tree->quadrant_1 = malloc(sizeof(Quadtree));
		*(tree->quadrant_1) = make_quadtree(N,
			tree->p1->x + (tree->p2->x - tree->p1->x)/2,
			tree->p1->y, 
			tree->p2->x,
			tree->p1->y + (tree->p2->y - tree->p1->y)/2
		);
		tree->quadrant_3 = malloc(sizeof(Quadtree));
		*(tree->quadrant_3) = make_quadtree(N,
			tree->p1->x,
			tree->p1->y + (tree->p2->y - tree->p1->y)/2,
			tree->p1->x + (tree->p2->x - tree->p1->x)/2,
			tree->p2->y
		);
		tree->quadrant_4 = malloc(sizeof(Quadtree));
		*(tree->quadrant_4) = make_quadtree(N,
			tree->p1->x + (tree->p2->x - tree->p1->x)/2, 
			tree->p1->y + (tree->p2->y - tree->p1->y)/2,
			tree->p2->x,
			tree->p2->y
		);
		// reassign stuff currently in tree->line into quadrants
		reassign_current_to_quadrants(tree);
	}
	// not leaf case
	// check where the line fits in
	if (l->p1->x >= tree->quadrant_1->p1->x &&
		l->p2->x <  tree->quadrant_1->p2->x &&
		l->p1->y >= tree->quadrant_1->p1->y &&
		l->p2->y <  tree->quadrant_1->p2->y) { // q1 case
		
		insert_line(l, tree->quadrant_1);
		return;
	} else if (l->p1->x >= tree->quadrant_2->p1->x &&
			   l->p2->x <  tree->quadrant_2->p2->x &&
			   l->p1->y >= tree->quadrant_2->p1->y &&
			   l->p2->y < tree->quadrant_2->p2->y) { // q2 case
		insert_line(l, tree->quadrant_2);
	} else if (l->p1->x >= tree->quadrant_3->p1->x &&
			   l->p2->x <  tree->quadrant_3->p2->x &&
			   l->p1->y >= tree->quadrant_3->p1->y &&
			   l->p2->y <  tree->quadrant_3->p2->y) { // q3 case
		insert_line(l, tree->quadrant_3);
	} else if (l->p1->x >= tree->quadrant_4->p1->x &&
			   l->p2->x <  tree->quadrant_4->p2->x &&
			   l->p1->y >= tree->quadrant_4->p1->y &&
			   l->p2->y <  tree->quadrant_4->p2->y) { // q4 case
		insert_line(l, tree->quadrant_4);
	} else { // must go into this node 
		// insert_line(l, tree);
		tree->lines[tree->numOfLines++] = l;
		return;
	}
}

void reassign_current_to_quadrants(Quadtree * tree) {
	int current_numOfLines = tree->numOfLines;
	for (int i = 0; i < current_numOfLines; i++) {
		// 1) pop before insert, shifting
		// 2) New Quadtree into which we insert and reassign pointers for quadtrees
		insert_line(tree->lines[i], tree);
	}
	// if(tree->numOfLines > current_numOfLines) { // stuff was reinserted.

	// }
}

// Recursively deletes all Quadtrees in this subtree
void delete_Quadtree(Quadtree * tree) {
	// not leaf
	free(&(tree->lines)); // leaf case
}
