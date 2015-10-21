/**
 * Copyright (c) 2015 the Massachusetts Institute of Technology
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 **/

#include "./IntersectionEventListReducer.h"

// Evaluates *left = *left OPERATOR *right.
// Cilk requires key, left, and right to be void * pointers
// We know left and right to be only IntersectionEventList * pointers
void IntersectionEventList_reduce(void* key, void* left, void* right) {
  assert(left);
  assert(right);

  IntersectionEventList * list1 = (IntersectionEventList *) left;
  IntersectionEventList * list2 = (IntersectionEventList *) right;
  
  // if list1 is empty, merge list2 into list1
  if (!(list1->head)) {
    list1->head = list2->head;
    list1->tail = list2->tail;
    list1->size = list2->size;
  } else if (list2->head) { // if list2 is not empty
    list1->tail->next = list2->head;
    list1->tail = list2->tail;
    list1->size += list2->size;
  }

  // if list2 is empty, do nothing

  // reset list2 to be empty
  list2->head = list2->tail = NULL;
  list2->size = 0;
}

// Sets *value to the the identity value.
void IntersectionEventList_identity(void* key, void* value) {
  *((IntersectionEventList *) value) = IntersectionEventList_make();
}

// Destroys any dynamically allocated memory.
void IntersectionEventList_destroy(void* key, void* value) {
  assert(value);

  IntersectionEventList_deleteNodes((IntersectionEventList *) value);
}
