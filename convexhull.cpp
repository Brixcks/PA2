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
	/* Add your code below */
	//make a local pair initialized to v[0]
    //search for the point given in step 1
    //set pair to found point
    //too lazy to write the rest since it's kinda obvious
    //smallest y-coordinate in the vector
    double minY = INFINITY;
    //smallest pair in the vector
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
    //Option 1 for sorting by angle
    //making a vector of the rest of the elements to run mergesort on
    vector<pair<double, double>> restofv;
    for (size_t i = 1; i < v.size(); i++) {
        restofv.push_back(v[i]);
    }

    //Option 2 for sorting by angle
}

//Mergesort
void Manglesort(vector<pair<double, double>>& vec, int lo, int hi) {
    
}

void Merge(vector<pair<double, double>>& vec, int lo, int hi) {

}

/**
 * Determines whether a path from p1 to p2 to p3 describes a counterclockwise turn
**/
bool CCW(pair<double, double> p1, pair<double, double> p2, pair<double, double> p3) {
	/* Replace the line below with your code */
    //refer to geeksforgeeks code found
    //access values with p1.first and p1.second...
	return false; // REPLACE THIS STUB
    double v = p1.first * (p2.second - p3.second) + 
               p2.first * (p3.second - p1.second) + 
               p3.first * (p1.second - p2.second);
    if (v < 0) return -1; 
    if (v > 0) return +1; 
    return 0;
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

	/* Add your code below */


	return hull;
}

