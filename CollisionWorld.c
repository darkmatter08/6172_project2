/** 
 * CollisionWorld.c -- detect and handle line segment intersections
 * Copyright (c) 2012 the Massachusetts Institute of Technology
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

#include "./CollisionWorld.h"

#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>

#include "./IntersectionDetection.h"
#include "./IntersectionEventList.h"
#include "./Line.h"
#include "./Quadtree.h"

CollisionWorld* CollisionWorld_new(const unsigned int capacity) {
  assert(capacity > 0);
  __cilkrts_set_param("nworkers", "8");

  CollisionWorld* collisionWorld = malloc(sizeof(CollisionWorld));
  if (collisionWorld == NULL) {
    return NULL;
  }

  collisionWorld->numLineWallCollisions = 0;
  collisionWorld->numLineLineCollisions = 0;
  collisionWorld->timeStep = 0.5;
  collisionWorld->lines = malloc(capacity * sizeof(Line*));
  collisionWorld->numOfLines = 0;
  return collisionWorld;
}

void CollisionWorld_delete(CollisionWorld* collisionWorld) {
  for (int i = 0; i < collisionWorld->numOfLines; i++) {
    free(collisionWorld->lines[i]);
  }
  free(collisionWorld->lines);
  free(collisionWorld);
}

unsigned int CollisionWorld_getNumOfLines(CollisionWorld* collisionWorld) {
  return collisionWorld->numOfLines;
}

void CollisionWorld_addLine(CollisionWorld* collisionWorld, Line *line) {
  collisionWorld->lines[collisionWorld->numOfLines] = line;
  collisionWorld->numOfLines++;
}

Line* CollisionWorld_getLine(CollisionWorld* collisionWorld,
                             const unsigned int index) {
  if (index >= collisionWorld->numOfLines) {
    return NULL;
  }
  return collisionWorld->lines[index];
}

void CollisionWorld_updateLines(CollisionWorld* collisionWorld) {
  CollisionWorld_detectIntersection(collisionWorld);
  CollisionWorld_updatePosition(collisionWorld);
  CollisionWorld_lineWallCollision(collisionWorld);
}

void CollisionWorld_updatePosition(CollisionWorld* collisionWorld) {
  double t = collisionWorld->timeStep;
  for (int i = 0; i < collisionWorld->numOfLines; i++) {
    Line *line = collisionWorld->lines[i];
    double dx = line->velocity.x * t;
    double dy = line->velocity.y * t;
    line->p1.x += dx;
    line->p1.y += dy;
    line->p2.x += dx;
    line->p2.y += dy;
  }
}

void CollisionWorld_lineWallCollision(CollisionWorld* collisionWorld) {
  for (int i = 0; i < collisionWorld->numOfLines; i++) {
    Line *line = collisionWorld->lines[i];
    bool collide = false;

    // Right side
    if ((line->p1.x > BOX_XMAX || line->p2.x > BOX_XMAX)
        && (line->velocity.x > 0)) {
      line->velocity.x = -line->velocity.x;
      collide = true;
    }
    // Left side
    if ((line->p1.x < BOX_XMIN || line->p2.x < BOX_XMIN)
        && (line->velocity.x < 0)) {
      line->velocity.x = -line->velocity.x;
      collide = true;
    }
    // Top side
    if ((line->p1.y > BOX_YMAX || line->p2.y > BOX_YMAX)
        && (line->velocity.y > 0)) {
      line->velocity.y = -line->velocity.y;
      collide = true;
    }
    // Bottom side
    if ((line->p1.y < BOX_YMIN || line->p2.y < BOX_YMIN)
        && (line->velocity.y < 0)) {
      line->velocity.y = -line->velocity.y;
      collide = true;
    }
    // Update total number of collisions.
    if (collide == true) {
      collisionWorld->numLineWallCollisions++;
    }
  }
}

void CollisionWorld_detectIntersection(CollisionWorld* collisionWorld) {
  // IntersectionEventList intersectionEventList = IntersectionEventList_make();

  IntersectionEventList_reducer X = CILK_C_INIT_REDUCER(
    /* type */ IntersectionEventList, IntersectionEventList_reduce, 
    IntersectionEventList_identity, IntersectionEventList_destroy,
    /* initial value */ IntersectionEventList_make()
  );

  CILK_C_REGISTER_REDUCER(X);

  // Replace as part of basic QuadTree
  Quadtree * tree = parse_CollisionWorld_to_Quadtree(collisionWorld);
  detect_collisions(tree, collisionWorld, &X);
  delete_Quadtree(tree);

  IntersectionEventList intersectionEventList = X.value;

  CILK_C_UNREGISTER_REDUCER(X);

  collisionWorld->numLineLineCollisions += intersectionEventList.size;

  // Sort the intersection event list.
  IntersectionEventNode* startNode = intersectionEventList.head;
  while (startNode != NULL) {
    IntersectionEventNode* minNode = startNode;
    IntersectionEventNode* curNode = startNode->next;
    while (curNode != NULL) {
      if (IntersectionEventNode_compareData(curNode, minNode) < 0) {
        minNode = curNode;
      }
      curNode = curNode->next;
    }
    if (minNode != startNode) {
      IntersectionEventNode_swapData(minNode, startNode);
    }
    startNode = startNode->next;
  }

  // Call the collision solver for each intersection event.
  IntersectionEventNode* curNode = intersectionEventList.head;

  while (curNode != NULL) {
    CollisionWorld_collisionSolver(collisionWorld, curNode->l1, curNode->l2,
                                   curNode->intersectionType);
    curNode = curNode->next;
  }

  IntersectionEventList_deleteNodes(&intersectionEventList);
}

unsigned int CollisionWorld_getNumLineWallCollisions(
    CollisionWorld* collisionWorld) {
  return collisionWorld->numLineWallCollisions;
}

unsigned int CollisionWorld_getNumLineLineCollisions(
    CollisionWorld* collisionWorld) {
  return collisionWorld->numLineLineCollisions;
}

void CollisionWorld_collisionSolver(CollisionWorld* collisionWorld,
                                    Line *l1, Line *l2,
                                    IntersectionType intersectionType) {
  assert(compareLines(l1, l2) < 0);
  assert(intersectionType == L1_WITH_L2
         || intersectionType == L2_WITH_L1
         || intersectionType == ALREADY_INTERSECTED);

  // Despite our efforts to determine whether lines will intersect ahead
  // of time (and to modify their velocities appropriately), our
  // simplified model can sometimes cause lines to intersect.  In such a
  // case, we compute velocities so that the two lines can get unstuck in
  // the fastest possible way, while still conserving momentum and kinetic
  // energy.
  if (intersectionType == ALREADY_INTERSECTED) {
    Vec p = getIntersectionPoint(l1->p1, l1->p2, l2->p1, l2->p2);

    if (Vec_length(Vec_subtract(l1->p1, p))
        < Vec_length(Vec_subtract(l1->p2, p))) {
      l1->velocity = Vec_multiply(Vec_normalize(Vec_subtract(l1->p2, p)),
                                  Vec_length(l1->velocity));
    } else {
      l1->velocity = Vec_multiply(Vec_normalize(Vec_subtract(l1->p1, p)),
                                  Vec_length(l1->velocity));
    }
    if (Vec_length(Vec_subtract(l2->p1, p))
        < Vec_length(Vec_subtract(l2->p2, p))) {
      l2->velocity = Vec_multiply(Vec_normalize(Vec_subtract(l2->p2, p)),
                                  Vec_length(l2->velocity));
    } else {
      l2->velocity = Vec_multiply(Vec_normalize(Vec_subtract(l2->p1, p)),
                                  Vec_length(l2->velocity));
    }
    return;
  }

  // Compute the collision face/normal vectors.
  Vec face;
  Vec normal;
  if (intersectionType == L1_WITH_L2) {
    Vec v = l2->relative_vector;//Vec_makeFromLine(*l2);
    face = Vec_normalize(v);
  } else {
    Vec v = l1->relative_vector;//Vec_makeFromLine(*l1);
    face = Vec_normalize(v);
  }
  normal = Vec_orthogonal(face);

  // Obtain each line's velocity components with respect to the collision
  // face/normal vectors.
  double v1Face = Vec_dotProduct(l1->velocity, face);
  double v2Face = Vec_dotProduct(l2->velocity, face);
  double v1Normal = Vec_dotProduct(l1->velocity, normal);
  double v2Normal = Vec_dotProduct(l2->velocity, normal);

  // Compute the mass of each line (we simply use its length).
  double m1 = Vec_length(l1->relative_vector);
  double m2 = Vec_length(l2->relative_vector);

  // Perform the collision calculation (computes the new velocities along
  // the direction normal to the collision face such that momentum and
  // kinetic energy are conserved).
  double newV1Normal = ((m1 - m2) / (m1 + m2)) * v1Normal
      + (2 * m2 / (m1 + m2)) * v2Normal;
  double newV2Normal = (2 * m1 / (m1 + m2)) * v1Normal
      + ((m2 - m1) / (m2 + m1)) * v2Normal;

  // Combine the resulting velocities.
  l1->velocity = Vec_add(Vec_multiply(normal, newV1Normal),
                         Vec_multiply(face, v1Face));
  l2->velocity = Vec_add(Vec_multiply(normal, newV2Normal),
                         Vec_multiply(face, v2Face));

  return;
}
