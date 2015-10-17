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
  Vec velocity = Vec_subtract(l2->velocity, l1->velocity);

  // Get the parallelogram.
  Vec p1 = Vec_add(l2->p1, Vec_multiply(velocity, time));
  Vec p2 = Vec_add(l2->p2, Vec_multiply(velocity, time));

  // l2 parallelogram: p1, p2, l2->p1, l2->p2
  // l2 bounding box:
  vec_dimension l2_tl_x = MIN(MIN(p1.x, p2.x), MIN(l2->p1.x, l2->p2.x));
  vec_dimension l2_tl_y = MIN(MIN(p1.y, p2.y), MIN(l2->p1.y, l2->p2.y));
  vec_dimension l2_br_x = MAX(MAX(p1.x, p2.x), MAX(l2->p1.x, l2->p2.x));
  vec_dimension l2_br_y = MAX(MAX(p1.y, p2.y), MAX(l2->p1.y, l2->p2.y));
  // l1 bounding box:
  vec_dimension l1_tl_x = MIN(l1->p1.x, l1->p2.x);
  vec_dimension l1_tl_y = MIN(l1->p1.y, l1->p2.y);
  vec_dimension l1_br_x = MAX(l1->p1.x, l1->p2.x);
  vec_dimension l1_br_y = MAX(l1->p1.y, l1->p2.y);
  
  // logic:
  if ( l1_br_x < l2_tl_x || l1_tl_x > l2_br_x || l1_br_y < l2_tl_y || l1_tl_y > l2_br_y )
    return NO_INTERSECTION;

  if (intersectLines(l1->p1, l1->p2, l2->p1, l2->p2)) {
    return ALREADY_INTERSECTED;
  }

  bool top_intersected = intersectLines(l1->p1, l1->p2, p1, l2->p1);
  bool bottom_intersected = intersectLines(l1->p1, l1->p2, p2, l2->p2);
  int num_line_intersections = top_intersected + bottom_intersected + intersectLines(l1->p1, l1->p2, p1, p2);

  if (num_line_intersections == 2) {
    return L2_WITH_L1;
  }

  if (pointInParallelogram(l1->p1, l2->p1, l2->p2, p1, p2)
      && pointInParallelogram(l1->p2, l2->p1, l2->p2, p1, p2)) {
    return L1_WITH_L2; // What's the difference wtih :67?
  }

  if (num_line_intersections == 0) {
    return NO_INTERSECTION;
  }

  double angle = Vec_angle(l1->relative_vector, l2->relative_vector);

  if (top_intersected) {
    if (angle < 0) {
      return L2_WITH_L1;
    } else {
      return L1_WITH_L2;
    }
  }

  if (bottom_intersected) {
    if (angle > 0) {
      return L2_WITH_L1;
    } else {
      return L1_WITH_L2;
    }
  }

  return L1_WITH_L2;
}

// Check if a point is in the parallelogram.
inline bool pointInParallelogram(Vec point, Vec p1, Vec p2, Vec p3, Vec p4) {
  double d1 = direction(p1, p2, point);
  double d2 = direction(p3, p4, point);
  double d3 = direction(p1, p3, point);
  double d4 = direction(p2, p4, point);

  if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0))
      && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
    return true;
  }
  return false;
}

// Check if two lines intersect.
inline bool intersectLines(Vec p1, Vec p2, Vec p3, Vec p4) {
  // Relative orientation
  double d1 = direction(p3, p4, p1);
  double d2 = direction(p3, p4, p2);
  double d3 = direction(p1, p2, p3);
  double d4 = direction(p1, p2, p4);

  // If (p1, p2) and (p3, p4) straddle each other, the line segments must
  // intersect.
  if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0))
      && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
    return true;
  }
  if (d1 == 0 && onSegment(p3, p4, p1)) {
    return true;
  }
  if (d2 == 0 && onSegment(p3, p4, p2)) {
    return true;
  }
  if (d3 == 0 && onSegment(p1, p2, p3)) {
    return true;
  }
  if (d4 == 0 && onSegment(p1, p2, p4)) {
    return true;
  }
  return false;
}

// Obtain the intersection point for two intersecting line segments.
Vec getIntersectionPoint(Vec p1, Vec p2, Vec p3, Vec p4) {
  double u;

  u = ((p4.x - p3.x) * (p1.y - p3.y) - (p4.y - p3.y) * (p1.x - p3.x))
      / ((p4.y - p3.y) * (p2.x - p1.x) - (p4.x - p3.x) * (p2.y - p1.y));

  return Vec_add(p1, Vec_multiply(Vec_subtract(p2, p1), u));
}

// Check the direction of two lines (pi, pj) and (pi, pk).
double direction(Vec pi, Vec pj, Vec pk) {
  return crossProduct(pk.x - pi.x, pk.y - pi.y, pj.x - pi.x, pj.y - pi.y);
}

// Check if a point pk is in the line segment (pi, pj).
// pi, pj, and pk must be collinear.
inline bool onSegment(Vec pi, Vec pj, Vec pk) {
  return (((pi.x <= pk.x && pk.x <= pj.x) || (pj.x <= pk.x && pk.x <= pi.x))
      && ((pi.y <= pk.y && pk.y <= pj.y) || (pj.y <= pk.y && pk.y <= pi.y)));
}

// Calculate the cross product.
double crossProduct(double x1, double y1, double x2, double y2) {
  return x1 * y2 - x2 * y1;
}

