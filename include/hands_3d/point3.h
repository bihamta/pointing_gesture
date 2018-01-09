#pragma once

#define UNCLASSIFIED -1
#define NOISE -2
#define CORE_POINT 1
#define NOT_CORE_POINT 0
#define SUCCESS 0
#define FAILURE -3

class Point3 {
	public:
		Point3() : x(0), y(0), z(0), cluster_id(UNCLASSIFIED) {};
		Point3(float x, float y, float z) : x(x), y(y), z(z), cluster_id(UNCLASSIFIED) {};
		Point3& operator=(const Point3& other) // copy assignment
		{
			x = other.x;
			y = other.y;
			z = other.z;
			return *this;
		}
		bool operator<(const Point3& other)
		{
			if (x != other.x) {
				return x < other.x;
			}
			if (y != other.y) {
				return y < other.y;
			}   
			return z < other.z;
		}
	public:
		float x, y, z;
		int cluster_id;
};
