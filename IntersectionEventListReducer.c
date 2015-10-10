#include "./IntersectionEventListReducer.h"

// Evaluates *left = *left OPERATOR *right.
void IntersectionEventList_reduce(void* key, void* left, void* right) {
	// IntersectionEventList * left_list = (IntersectionEventList *) left;
	// IntersectionEventList * right_list = (IntersectionEventList *) right;
	// left_list->tail->next = right_list->head;
	// left_list->tail = right_list->tail;
	// right_list->head = right_list->tail = NULL;


	///
	IntersectionEventList * list1 = (IntersectionEventList *) left;
	IntersectionEventList * list2 = (IntersectionEventList *) right;
	if (!(list1->head)) { // nothing in list1
    list1->head = list2->head;
    list1->tail = list2->tail;
    // list1->size = list2->size;
	}
	else if (!(list2->head)) { // nothing in list2
	  ; // do nothing
	}
	else { // things in both list1 and list2
	  list1->tail->next = list2->head;
	  list1->tail = list2->tail;
	  // list1->size += list2->size;
	}
	// reset list2
	list2->head = list2->tail = NULL;
	// list2->size = 0;
}

// Sets *value to the the identity value.
void IntersectionEventList_identity(void* key, void* value) {
	*((IntersectionEventList *) value) = IntersectionEventList_make();
}

// Destroys any dynamically allocated memory.
void IntersectionEventList_destroy(void* key, void* value) {
	IntersectionEventList_deleteNodes((IntersectionEventList *) value);
}


// IntersectionEventList_reducer X = CILK_C_INIT_REDUCER(
// 	/* type */ IntersectionEventList, IntersectionEventList_reduce, 
// 	IntersectionEventList_identity, IntersectionEventList_destroy,
// 	/* initial value */ (IntersectionEventList) { .head = NULL, .tail = NULL }
// );

// CILK_C_REGISTER_REDUCER(X);

// REDUCER_VIEW(X);

// CILK_C_UNREGISTER_REDUCER(X);