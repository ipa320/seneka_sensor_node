#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>

struct Point2D {
	int x;
	int y;
	Point2D(int new_x, int new_y) : x(new_x), y(new_y) {}
};

//function declarations
void addCirclePoints(std::vector< std::vector<Point2D> >& octants, int x, int y);
std::vector<Point2D> bresenhamCircle(int radius_in_cells);
std::vector<Point2D> bresenhamLine(int x_end, int y_end, int x_start = 0, int y_start = 0);
std::vector< std::vector<Point2D> > pointsInsideCircle(int radius);

int main()
{
	//test with circle of radius 4:

	//get points
	std::vector< std::vector<Point2D> > all_points = pointsInsideCircle(4);

	//output: one line for each ray
	for(unsigned int i=0; i<all_points.size(); i++)
	{
		for(unsigned int k=0; k<all_points[i].size(); k++)
		{
			std::cout << "(" << all_points[i][k].x << "/" << all_points[i][k].y << ")  ";
		}
		std::cout << std::endl;
	}

	return 0;
}


//adds a point and its mirrors to the according vectors
void addCirclePoints(std::vector< std::vector<Point2D> >& octants, int x, int y)
{
	octants[0].push_back(Point2D(x,y));
	octants[1].push_back(Point2D(y,x));
	octants[2].push_back(Point2D(-y,x));
	octants[3].push_back(Point2D(-x,y));
	octants[4].push_back(Point2D(-x,-y));
	octants[5].push_back(Point2D(-y,-x));
	octants[6].push_back(Point2D(y,-x));
	octants[7].push_back(Point2D(x,-y));
}

//returns a vector of points creating the circle with the given radius (>3)
//uses midpoint circle algorithm
std::vector<Point2D> bresenhamCircle(int radius_in_cells)
{
	//vector storing the circle points divided in eight parts of the circle due to symmetry
	std::vector< std::vector<Point2D> > octants;
	octants.resize(8);

	int x,y;
	int error;

	x = radius_in_cells;
	y = 0;
	error = 1 - radius_in_cells;

	while(x >= y)
	{
		//add current point and mirrors to the vectors
		addCirclePoints(octants,x,y);
		y++;

		if(error < 0)
		{
			error += y * 2 + 1;
		}
		else
		{
			x--;
			error += 2 * (y - x + 1);

			//if x is decreased, also add the point next to it to ensure 4-connectivity
			addCirclePoints(octants,x+1,y); 
		}	
	}

	//vector for all points of the circle in correct order
	std::vector<Point2D> circle_points;

	//for every octant
	for(unsigned int i=0; i<8; i++)
	{
		if(i%2 != 0)
		{
			//flip every second octant vector
			std::reverse(octants[i].begin(),octants[i].end());
		}

		//delete last element and add to vector for the complete circle
		octants[i].pop_back();
		circle_points.insert(circle_points.end(), octants[i].begin(), octants[i].end());
	}

	return circle_points;
}

//returns a vector of points which are travelled by a ray between the start and end point
//uses bresenham algorithm with 4-connectivity
std::vector<Point2D> bresenhamLine(int x_end, int y_end, int x_start, int y_start)
{
	//absolute distances
	unsigned int dx = std::abs(x_end - x_start);
	unsigned int dy = std::abs(y_end - y_start);
	unsigned int number_of_points = dx+dy;

	//all cells in the ray
	std::vector<Point2D> ray_points;

	//increase or decrease x,y?
	int x_inc, y_inc;

	if(x_start < x_end)
		x_inc = 1;
	else
		x_inc = -1;

	if(y_start < y_end)
		y_inc = 1;
	else
		y_inc = -1;

	//current position in ray
	int x = x_start;
	int y = y_start;

	//bresenham errors
	int error = 0;
	int e_x, e_y;

	//add first cell
	ray_points.push_back(Point2D(x,y));

	//make 1 step in either x or y direction
	for(unsigned int i=0; i<number_of_points; i++)
	{
		e_x = error - dx;
		e_y = error + dy;

		if(std::abs(e_y) < std::abs(e_x))
			//error is smaller if moving in x direction
		{
			x += x_inc;
			error = e_y;
		}
		else
			//error is smaller moving in y direction
		{
			y += y_inc;
			error = e_x;
		}

		//add actual position
		ray_points.push_back(Point2D(x,y));
	}

	return ray_points;
}

//returns a vector of rays to all points on the circle with the given radius
std::vector< std::vector<Point2D> > pointsInsideCircle(int radius)
{
	//get points on the circle
	std::vector<Point2D> circle = bresenhamCircle(radius);
	unsigned int number_of_rays = circle.size();

	//final vector containing all rays
	std::vector< std::vector<Point2D> > points_inside_circle;
	points_inside_circle.resize(number_of_rays);

	//for every point on the circle 'create' one ray
	for(unsigned int i=0; i<number_of_rays; i++)
	{
		points_inside_circle[i] = bresenhamLine(circle[i].x,circle[i].y);
	}

	return points_inside_circle;
}




