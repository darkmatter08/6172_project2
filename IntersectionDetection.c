/**
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

#include "./IntersectionDetection.h"

// Detect if lines l1 and l2 will intersect between now and the next time step.
IntersectionType intersect(Line *l1, Line *l2, double time) {
  assert(compareLines(l1, l2) < 0);

  // Get relative velocity.
  double dx = (l2->velocity.x - l1->velocity.x) * time;
  double dy = (l2->velocity.y - l1->velocity.y) * time;

  // Get the parallelogram.
  Vec p1 = {.x = l2->p1.x + dx, .y = l2->p1.y + dy};
  Vec p2 = {.x = l2->p2.x + dx, .y = l2->p2.y + dy};

  double d1 = direction(l1->p1, l1->p2, l2->p1);
  double d2 = direction(l1->p1, l1->p2, l2->p2);
  double d3 = direction(l1->p1, l1->p2, p1);
  double d4 = direction(l1->p1, l1->p2, p2);

  if ((d1 * d2 > 0) && (d2 * d3 > 0) && (d3 * d4 > 0)) {
    return NO_INTERSECTION;
  }

  double d5 = direction(l2->p1, l2->p2, l1->p1);
  double d6 = direction(l2->p1, l2->p2, l1->p2); 

  if (intersectLines(l1->p1, l1->p2, l2->p1, l2->p2, d1*d2, d5*d6)) {
    return ALREADY_INTERSECTED;
  }

  double d7 = direction(p1, l2->p1, l1->p1);
  double d8 = direction(p1, l2->p1, l1->p2);
  double d9 = direction(p2, l2->p2, l1->p1);
  double d10 = direction(p2, l2->p2, l1->p2); 
  double d11 = direction(p1, p2, l1->p1);
  double d12 = direction(p1, p2, l1->p2);

  bool top_intersected = intersectLines(l1->p1, l1->p2, p1, l2->p1, d3*d1, d7*d8);
  bool bottom_intersected = intersectLines(l1->p1, l1->p2, p2, l2->p2, d4*d2, d9*d10);
  int num_line_intersections = top_intersected + bottom_intersected + intersectLines(l1->p1, l1->p2, p1, p2, d3*d4, d11*d12);

  if (num_line_intersections == 2) {
    return L2_WITH_L1;
  }

  if (pointInParallelogram(l1->p1, l2->p1, l2->p2, p1, p2, d5, d11, -d7, -d9)
      && pointInParallelogram(l1->p2, l2->p1, l2->p2, p1, p2, d6, d12, -d8, -d10)) {
    return L1_WITH_L2; // What's the difference wtih :67?
  }

  if (num_line_intersections == 0) {
    return NO_INTERSECTION;
  }

  double angle = Vec_angle(l1->relative_vector, l2->relative_vector);

  if ((top_intersected && (angle < 0)) || (bottom_intersected && (angle > 0))) {
    return L2_WITH_L1;
  }

  return L1_WITH_L2;
}

// Check if a point is in the parallelogram.
inline bool pointInParallelogram(Vec point, Vec p1, Vec p2, Vec p3, Vec p4, double d1, double d2, double d3, double d4) {
  return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0))
      && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
}

// Check if two lines intersect.
inline bool intersectLines(Vec p1, Vec p2, Vec p3, Vec p4, double d1, double d2) {
  // If (p1, p2) and (p3, p4) straddle each other, the line segments must
  // intersect.
  if (d1 < 0 && d2 < 0) {
    return true;
  }
  if (d1 * d2 != 0) {
    return false;
  }
  return onSegment(p3, p4, p1) || onSegment(p3, p4, p2) || onSegment(p1, p2, p3) || onSegment(p1, p2, p4);
}

// Obtain the intersection point for two intersecting line segments.
Vec getIntersectionPoint(Vec p1, Vec p2, Vec p3, Vec p4) {
  double u = ((p4.x - p3.x) * (p1.y - p3.y) - (p4.y - p3.y) * (p1.x - p3.x))
      / ((p4.y - p3.y) * (p2.x - p1.x) - (p4.x - p3.x) * (p2.y - p1.y));

  return (Vec) {.x = p1.x + (p2.x - p1.x) * u, .y = p1.y + (p2.y - p1.y) * u};
}

// Check the direction of two lines (pi, pj) and (pi, pk).
inline double direction(Vec pi, Vec pj, Vec pk) {
  return (pk.x - pi.x) * (pj.y - pi.y) - (pk.y - pi.y) * (pj.x - pi.x);
}

// Check if a point pk is in the line segment (pi, pj).
// pi, pj, and pk must be collinear.
inline bool onSegment(Vec pi, Vec pj, Vec pk) {
  return (((pi.x <= pk.x && pk.x <= pj.x) || (pj.x <= pk.x && pk.x <= pi.x))
      && ((pi.y <= pk.y && pk.y <= pj.y) || (pj.y <= pk.y && pk.y <= pi.y)));
}