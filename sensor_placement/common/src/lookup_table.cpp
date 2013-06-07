#include <vector>
#include <iostream>
#include <cmath>

struct Point2D {
	int x;
	int y;
	Point2D(int new_x, int new_y) : x(new_x), y(new_y) {}
};

//basic bresenham algorithm, but 4-connected instead of 8-connected so every travelled cell is marked
std::vector<Point2D> bresenham4Connected(int x_start, int y_start, int x_end, int y_end)
{
	//absolute distances
	int dx = std::abs(x_end - x_start);
	int dy = std::abs(y_end - y_start);
	int number_of_points = dx+dy;

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
	for(int i=0; i<number_of_points; i++)
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

int main()
{
	unsigned int number_of_cells = 5;

	for(unsigned int i=0; i < number_of_cells; i++)
	{
		std::vector<Point2D> actual_ray = bresenham4Connected(0,0,number_of_cells,i);

		std::cout << "Ray Nr." << i+1 << " :" << std::endl;
		std::cout << "*********" << std::endl;

		for(unsigned int k=0; k<actual_ray.size(); k++)
		{
			std::cout << "x: " << actual_ray[k].x << "   y: " << actual_ray[k].y << std::endl;
		}

		std::cout << "*********" << std::endl;
	}	

	return 0;
}



