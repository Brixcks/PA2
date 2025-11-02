/**
 * @file convexhull.cpp
 * @description Implementation of methods used for computing convex hull using
 *              Graham scan algorithm
 *              https://en.wikipedia.org/wiki/Graham_scan
 *
 * @author      CPSC 221: Basic Algorithms and Data Structures
 *
 * THIS FILE MUST BE SUBMITTED
**/

#include "convexhull.h"

/***************************************************
* IF YOU WISH TO DEFINE YOUR OWN CUSTOM FUNCTIONS, *
* ADD THEM HERE                                    *
***************************************************/



/**
 * Uses a sorting algorithm of your choice(must be fully implemented by you)
 * to sort the points in v to satisfy the following criteria:
 * 1. v[0] must contain the point with the smallest y-coordinate.
 *      If multiple points have the same smallest y-coordinate, v[0]
 *      must contain the one among those with the smallest x-coordinate.
 * 2. The remaining indices i contain the points, sorted in increasing order
 *      by the angle that the point forms with v[0] and the x-axis. THat is,
 * 	one of the legs of the angle is represened by the line through v[0] and
 *	v[i], and the other is the x-axis.
 * NOTE: "smallest" y-coordinate is actually closest to the TOP of a PNG image.
**/
void SortByAngle(vector<pair<double, double>>& v) {
    //smallest y-coordinate in the vector
    double minY = INFINITY;
    //smallest pair in the vector intialized with v[0] as a dummy value
    pair<double, double> minPair = v[0];
    //index of the chose pair
    int mindex = 0;
    for (size_t i = 0; i < v.size(); i++) {
        if (v[i].second < minY) {
            minPair = v[i];
            minY = v[i].second;
            mindex = i;
        } else if (v[i].second == minY) {
            if (v[i].first < minPair.first) {
                minPair = v[i];
                minY = v[i].second;
                mindex = i;
            }
        }
    }
    pair<double, double> swapling = v[0];
    v[0] = minPair;
    v[mindex] = swapling;
    //mergesort by angle relative to v[0]
    Manglesort(v);
}

//applies mergesort over everything past v[0]
void Manglesort(vector<pair<double, double>>& v) {
    Msort(v, 1, v.size() - 1);
}

//Mergesort comparing by angle with respect to v[0]
void Msort(vector<pair<double, double>>& v, int lo, int hi) {
    if (lo < hi) {
        int mid = (lo + hi)/2;
        Msort(v, lo, mid);
        Msort(v, mid + 1, hi);
        Mangle(v, lo, mid, hi);
    }
}

//gets the angle between v[0] and the given point in degrees
float getAngle(pair<double, double> p1, pair<double, double> p2) {
    double deltaX = (p2.first - p1.first);
    double deltaY = (p2.second - p1.second);
    float angle = atan2(deltaY, deltaX) * 180 / 3.14159;
    //return the inverted angle due to y being inverted
    return (angle * -1);
}

//Merges subarrays while sorting by angle with respect to v[0]
void Mangle(vector<pair<double, double>>& v, int lo, int mid, int hi) {
    vector<pair<double, double>> temp;
    int a = lo;
    int b = mid + 1;
    for (int i = lo; i <= hi; i++) {
        //calculate angles for a and b with respect to v[0]
        float angleA = getAngle(v[0], v[a]);
        float angleB = getAngle(v[0], v[b]);
        if (a <= mid && (b > hi || angleA < angleB)) {
            temp.push_back(v[a++]);
        } else {
            temp.push_back(v[b++]);
        }
    }
    for (int j = lo; j <= hi; j++) {
        v[j] = temp[j-lo];
    }
}

/**
 * Determines whether a path from p1 to p2 to p3 describes a counterclockwise turn
**/
bool CCW(pair<double, double> p1, pair<double, double> p2, pair<double, double> p3) {
    //access values with p1.first and p1.second...
    double v = p1.first * (p2.second - p3.second) + 
               p2.first * (p3.second - p1.second) + 
               p3.first * (p1.second - p2.second);
    if (v < 0) {
        return true;
    } else {
        return false;
    }
}

/**
 * Returns a vector of points representing the convex hull of v
 * if c is the vector representing the convex hull, then c[k], c[k+1]
 * is an edge in the convex hull, for all k in 0 to n-1, where n is the
 * number of points in the hull. c[n-1],c[0] is also an edge in the hull.
 * Note that only the endpoints of a colinear sequence will belong in the hull.
 * The returned vector should be a subset of v
 * Input: v - a vector of points in arbitrary order
**/
vector<pair<double, double>> GetConvexHull(vector<pair<double, double>>& v) {
	vector<pair<double, double>> hull;
    SortByAngle(v);
	/* Add your code below */


	return hull;
}
