#include <"./Quadtree.h">

// Make new Quadtree, lines and points not set.
Quadtree make_quadtree(unsigned int capacity, double x_lo, double y_lo, 
	double x_hi, double y_hi) {
	Quadtree new_tree = {
		.quadrant_1 = NULL, .quadrant_2 = NULL, .quadrant_3 = NULL,
		.quadrant_4 = NULL, .line = malloc(sizeof(Line *) * capacity), // TODO:CHECK
		.numOfLines = 0, .p1 = *((Vec *) malloc(sizeof(Vec))), // TODO:CHECK
		.p2 = *((Vec *) malloc(sizeof(Vec))) // TODO:CHECK
	};
	new_tree->p1->x = x_lo; //x
	new_tree->p1->y = y_lo;
	new_tree->p2->x = x_hi;
	new_tree->p2->y = y_hi;
	return new_tree;
}

// NOTE: Two strategies for this method: 
// 1) Parse lines from world one-by-one
// 2) Take all lines from world, fix the data structure until it works. 
// Parses the CollisionWorld into a Quadtree
Quadtree* parse_CollisionWord_to_Quadtree(CollisionWord * world) {
	Quadtree tree = make_quadtree(N); // need some type of malloc here.
	for (int i = 0; i < world->numOfLines; i++) {
		Line *l = word->lines[i];
		insert_line(l, tree);
	}
	return &tree;
}

// Insert into the subtree hanging from (and including) tree
void insert_line(Line* l, Quadtree * tree) {
	if (tree->numOfLines < N && tree->quadrant_1 == NULL) { // not-full leaf
		tree->lines[tree->numOfLines++] = l;
		return;
	} else if (tree->quadrant_1 == NULL) { // full leaf
		tree->quadrant_2 = make_quadtree(N,
			tree->p1->x,
			tree->p1->y, 
			tree->p1->x + (tree->p2->x - tree->p1->x)/2, 
			tree->p1->y + (tree->p2->y - tree->p1->y)/2
		);
		tree->quadrant_1 = make_quadtree(N,
			tree->p1->x + (tree->p2->x - tree->p1->x)/2,
			tree->p1->y, 
			tree->p2->x,
			tree->p1->y + (tree->p2->y - tree->p1->y)/2
		);
		tree->quadrant_3 = make_quadtree(N,
			tree->p1->x,
			tree->p1->y + (tree->p2->y - tree->p1->y)/2,
			tree->p1->x + (tree->p2->x - tree->p1->x)/2,
			tree->p2->y
		);
		tree->quadrant_4 = make_quadtree(N,
			tree->p1->x + (tree->p2->x - tree->p1->x)/2, 
			tree->p1->y + (tree->p2->y - tree->p1->y)/2,
			tree->p2->x,
			tree->p2->y
		);
		// reassign stuff currently in tree->line into quadrants
		reassign_current_to_quadrants(world);
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
		insert_line(l, tree);
	}
}

void reassign_current_to_quadrants(CollisionWord * world) {

}

// Recursively deletes all Quadtrees in this subtree
void delete_Quadtree(Quadtree * tree) {

}
