#include <cilk/reducer.h>
#include "./IntersectionEventList.h"

// Evaluates *left = *left OPERATOR *right.
void IntersectionEventList_reduce(void* key, void* left, void* right);

// Sets *value to the the identity value.
void IntersectionEventList_identity(void* key, void* value);

// Destroys any dynamically allocated memory. Hint: delete_nodes.
void IntersectionEventList_destroy(void* key, void* value);

typedef CILK_C_DECLARE_REDUCER(IntersectionEventList) IntersectionEventList_reducer;
