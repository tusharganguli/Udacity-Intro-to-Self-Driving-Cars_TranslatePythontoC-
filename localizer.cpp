/**
	localizer.cpp

	Purpose: implements a 2-dimensional histogram filter
	for a robot living on a colored cyclical grid by 
	correctly implementing the "initialize_beliefs", 
	"sense", and "move" functions.

	This file is incomplete! Your job is to make these
	functions work. Feel free to look at localizer.py 
	for working implementations which are written in python.
*/

#include <stdlib.h>

#include "localizer.h"
#include "helpers.h"
#include "debugging_helpers.h"

using namespace std;

typedef vector<float> v_float;
typedef vector<char> v_char;
typedef vector< v_float > vv_float;
typedef vector< v_char > vv_char;

/**
	TODO - implement this function 
    
    Initializes a grid of beliefs to a uniform distribution. 

    @param grid - a two dimensional grid map (vector of vectors 
    	   of chars) representing the robot's world. For example:
    	   
    	   g g g
    	   g r g
    	   g g g
		   
		   would be a 3x3 world where every cell is green except 
		   for the center, which is red.

    @return - a normalized two dimensional grid of floats. For 
           a 2x2 grid, for example, this would be:

           0.25 0.25
           0.25 0.25
*/
vv_float initialize_beliefs(vv_char grid) {
	vv_char::size_type height = grid.size();
	v_char::size_type width = grid[0].size();
	v_char::size_type area = height * width;
	float belief_per_cell = static_cast<float>(1.0 / area);
	vv_float newGrid(height, v_float(width, belief_per_cell));
	
	return newGrid;
}

/**
  TODO - implement this function 
    
    Implements robot motion by updating beliefs based on the 
    intended dx and dy of the robot. 

    For example, if a localized robot with the following beliefs

    0.00  0.00  0.00
    0.00  1.00  0.00
    0.00  0.00  0.00 

    and dx and dy are both 1 and blurring is 0 (noiseless motion),
    than after calling this function the returned beliefs would be

    0.00  0.00  0.00
    0.00  0.00  0.00
    0.00  0.00  1.00 

  @param dy - the intended change in y position of the robot

  @param dx - the intended change in x position of the robot

    @param beliefs - a two dimensional grid of floats representing
         the robot's beliefs for each cell before sensing. For 
         example, a robot which has almost certainly localized 
         itself in a 2D world might have the following beliefs:

         0.01 0.98
         0.00 0.01

    @param blurring - A number representing how noisy robot motion
           is. If blurring = 0.0 then motion is noiseless.

    @return - a normalized two dimensional grid of floats 
         representing the updated beliefs for the robot. 
*/
vv_float move(int dy, int dx, vv_float beliefs, float blurring) {

	size_t rows = beliefs.size();
	size_t cols = beliefs[0].size();
	vv_float newGrid(rows,v_float(cols,0));

	for (size_t i = 0; i < rows; i++) {
		for (size_t j = 0; j < cols; j++) {
			newGrid[(i + dy) % rows][(j + dx) % cols] = beliefs[i][j];
		}
	}
	return blur(newGrid, blurring);
}


/**
	TODO - implement this function 
    
    Implements robot sensing by updating beliefs based on the 
    color of a sensor measurement 

	@param color - the color the robot has sensed at its location

	@param grid - the current map of the world, stored as a grid
		   (vector of vectors of chars) where each char represents a 
		   color. For example:

		   g g g
    	   g r g
    	   g g g

   	@param beliefs - a two dimensional grid of floats representing
   		   the robot's beliefs for each cell before sensing. For 
   		   example, a robot which has almost certainly localized 
   		   itself in a 2D world might have the following beliefs:

   		   0.01 0.98
   		   0.00 0.01

    @param p_hit - the RELATIVE probability that any "sense" is 
    	   correct. The ratio of p_hit / p_miss indicates how many
    	   times MORE likely it is to have a correct "sense" than
    	   an incorrect one.

   	@param p_miss - the RELATIVE probability that any "sense" is 
    	   incorrect. The ratio of p_hit / p_miss indicates how many
    	   times MORE likely it is to have a correct "sense" than
    	   an incorrect one.

    @return - a normalized two dimensional grid of floats 
    	   representing the updated beliefs for the robot. 
*/
vv_float sense(	char color, vv_char grid, vv_float beliefs, 
				float p_hit, float p_miss) 
{

	size_t rows = beliefs.size();
	size_t cols = beliefs[0].size();
	vv_float newBeliefs(rows, v_float(cols, 0));

	// calculate the new beliefs 
	for (size_t i = 0; i < rows; i++) {
		for (size_t j = 0; j < cols; j++) {
			int hit = (grid[i][j] == color);
			newBeliefs[i][j] = beliefs[i][j] * (p_hit * (hit)+ p_miss*(1-hit));
		}
	}
	return normalize(newBeliefs);
}
