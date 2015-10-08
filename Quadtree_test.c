#include "./Quadtree.h"

Quadtree* create_Quadtree();
void insert_1(Quadtree *tree);
void insert_4(Quadtree *tree);
void check_line_equality(Line * line1, Line * line2);
void insert_4_spanning_quadrants(Quadtree *tree);
void insert_4_same_quadrant_and_subquadrant();

#ifdef TEST
int main(int argc, char *argv[]) {
	insert_1(create_Quadtree());
	printf("Passed insert_1\n");
	insert_4(create_Quadtree());
	printf("Passed insert_4\n");
	insert_4_spanning_quadrants(create_Quadtree());
	printf("Passed insert_4_spanning_quadrants\n");
	insert_4_same_quadrant_and_subquadrant(create_Quadtree());
	printf("Passed insert_4_same_quadrant_and_subquadrant\n");
	return 0;
}
#endif

Quadtree * create_Quadtree() {
	Quadtree *tree = malloc(sizeof(Quadtree));
	*tree = make_quadtree(N, 0, 0, 6, 6);
	
	// check bounds
	assert(tree->p1.x == 0);
	assert(tree->p1.y == 0);
	assert(tree->p2.x == 6);
	assert(tree->p2.y == 6);

	assert(tree->numOfLines == 0);
	assert(tree->capacity == N);
	
	// check quadrants
	assert(tree->quadrant_1 == NULL);
	assert(tree->quadrant_2 == NULL);
	assert(tree->quadrant_3 == NULL);
	assert(tree->quadrant_4 == NULL);
	
	return tree;
}

void insert_1(Quadtree *tree) {
	Line l1 = { .p1 = { .x = 0.5, .y = 0.5 }, .p2 = { .x = 1, .y = 1 },
				.velocity = { .x = 5, .y = 5 }, .color = RED, .id = 1 };
	insert_line(&l1, tree);

	assert(tree->numOfLines == 1);
	assert(tree->quadrant_1 == NULL);
	assert(tree->quadrant_2 == NULL);
	assert(tree->quadrant_3 == NULL);
	assert(tree->quadrant_4 == NULL);
	delete_Quadtree(tree);
}

void insert_4(Quadtree *tree) {
	Line l1 = { .p1 = { .x = 0.5, .y = 0.5 }, .p2 = { .x = 1, .y = 1 },
				.velocity = { .x = 5, .y = 5 }, .color = RED, .id = 1 };
	insert_line(&l1, tree);
	insert_line(&l1, tree);
	insert_line(&l1, tree);

	assert(tree->numOfLines == 3);
	assert(tree->quadrant_1 == NULL);
	assert(tree->quadrant_2 == NULL);
	assert(tree->quadrant_3 == NULL);
	assert(tree->quadrant_4 == NULL);

	// insert a 4th line, should NOT enter into Q2
	Line l2 = { .p1 = { .x = 4, .y = 4 }, .p2 = { .x = 5, .y = 5 }, // Q4
				.velocity = { .x = 5, .y = 5 }, .color = RED, .id = 1 };
	insert_line(&l2, tree);
	// root node asserts
	assert(tree->numOfLines == 0);
	assert(tree->quadrant_1 != NULL);
	assert(tree->quadrant_2 != NULL);
	assert(tree->quadrant_3 != NULL);
	assert(tree->quadrant_4 != NULL);

	// check bounds of quadrants
	//// q1
	assert(tree->quadrant_1->p1.x == 3);
	assert(tree->quadrant_1->p1.y == 0);
	assert(tree->quadrant_1->p2.x == 6);
	assert(tree->quadrant_1->p2.y == 3);
	//// q2
	assert(tree->quadrant_2->p1.x == 0);
	assert(tree->quadrant_2->p1.y == 0);
	assert(tree->quadrant_2->p2.x == 3);
	assert(tree->quadrant_2->p2.y == 3);
	//// q3
	assert(tree->quadrant_3->p1.x == 0);
	assert(tree->quadrant_3->p1.y == 3);
	assert(tree->quadrant_3->p2.x == 3);
	assert(tree->quadrant_3->p2.y == 6);
	//// q4
	assert(tree->quadrant_4->p1.x == 3);
	assert(tree->quadrant_4->p1.y == 3);
	assert(tree->quadrant_4->p2.x == 6);
	assert(tree->quadrant_4->p2.y == 6);

	/////////// not filled quadrant asserts ///////////
	// check that no lines were inserted into q1, q3, or q4
	assert(tree->quadrant_1->numOfLines == 0);
	assert(tree->quadrant_3->numOfLines == 0);

	///////////   filled quadrant asserts   ///////////
	assert(tree->quadrant_2->numOfLines == 3);
	assert(tree->quadrant_4->numOfLines == 1);

	// check stored line is the same
	check_line_equality(*(tree->quadrant_4->lines), &l2);
	// assert(tree->quadrant_4->lines[0]->p1.x == 4);
	// assert(tree->quadrant_4->lines[0]->p1.y == 4);
	// assert(tree->quadrant_4->lines[0]->p2.x == 5);
	// assert(tree->quadrant_4->lines[0]->p2.y == 5);

	check_line_equality(tree->quadrant_2->lines[0], &l1);
	check_line_equality(tree->quadrant_2->lines[1], &l1);
	check_line_equality(tree->quadrant_2->lines[2], &l1);
	delete_Quadtree(tree);
}

void check_line_equality(Line * line1, Line * line2) {
	assert(line1->p1.x == line2->p1.x);
	assert(line1->p1.y == line2->p1.y);
	assert(line1->p2.x == line2->p2.x);
	assert(line1->p2.y == line2->p2.y);

	assert(line1->velocity.x == line2->velocity.x);
	assert(line1->velocity.y == line2->velocity.y);
	
	assert(line1->color == line2->color);

	assert(line1->id == line2->id);
}

void insert_4_spanning_quadrants(Quadtree *tree) {
	;
}

// tree->quandrant_2 further divides into quadrants because of 
void insert_4_same_quadrant_and_subquadrant(Quadtree *tree) {
	Line l1 = { .p1 = { .x = 0.5, .y = 0.5 }, .p2 = { .x = 1, .y = 1 }, // Q2->Q2
				.velocity = { .x = 5, .y = 5 }, .color = RED, .id = 1 };
	insert_line(&l1, tree);
	insert_line(&l1, tree);
	insert_line(&l1, tree);

	assert(tree->numOfLines == 3);
	assert(tree->quadrant_1 == NULL);
	assert(tree->quadrant_2 == NULL);
	assert(tree->quadrant_3 == NULL);
	assert(tree->quadrant_4 == NULL);

	// insert a 4th line, should enter into Q2
	Line l2 = { .p1 = { .x = 2, .y = 2 }, .p2 = { .x = 2.5, .y = 2.5 }, // Q2->Q4
				.velocity = { .x = 5, .y = 5 }, .color = RED, .id = 1 };
	insert_line(&l2, tree);
	// root node asserts
	assert(tree->numOfLines == 0);
	assert(tree->quadrant_1 != NULL);
	assert(tree->quadrant_2 != NULL);
	assert(tree->quadrant_3 != NULL);
	assert(tree->quadrant_4 != NULL);

	// check bounds of quadrants inside tree->quandrant_2
	//// q1
	assert(tree->quadrant_2->quadrant_1->p1.x == 1.5);
	assert(tree->quadrant_2->quadrant_1->p1.y == 0);
	assert(tree->quadrant_2->quadrant_1->p2.x == 3);
	assert(tree->quadrant_2->quadrant_1->p2.y == 1.5);
	//// q2
	assert(tree->quadrant_2->quadrant_2->p1.x == 0);
	assert(tree->quadrant_2->quadrant_2->p1.y == 0);
	assert(tree->quadrant_2->quadrant_2->p2.x == 1.5);
	assert(tree->quadrant_2->quadrant_2->p2.y == 1.5);
	//// q3
	assert(tree->quadrant_2->quadrant_3->p1.x == 0);
	assert(tree->quadrant_2->quadrant_3->p1.y == 1.5);
	assert(tree->quadrant_2->quadrant_3->p2.x == 1.5);
	assert(tree->quadrant_2->quadrant_3->p2.y == 3);
	//// q4
	assert(tree->quadrant_2->quadrant_4->p1.x == 1.5);
	assert(tree->quadrant_2->quadrant_4->p1.y == 1.5);
	assert(tree->quadrant_2->quadrant_4->p2.x == 3);
	assert(tree->quadrant_2->quadrant_4->p2.y == 3);

	///////////   filled quadrant asserts   ///////////
	/// TODO 	
	delete_Quadtree(tree);
}