#pragma once

#include <cmath>
#include <cstdlib>
#include <vector>
#include "hands_3d/point3.h"

class DBSCAN {
public:
	class Node {
	public:
		int index;
		DBSCAN::Node *next;
	public:
		Node() : next(NULL) {};
	};

	class EpsilonNeighbours {
	public:
		unsigned int num_members;
		Node *head, *tail;
	public:
		EpsilonNeighbours() : head(NULL), tail(NULL) {};
	};

	class Cluster {
	public:
		std::vector<Point3*> points;
		int cluster_id;
	};

public:
	std::vector<Cluster> cluster(
			const std::vector<Point3*>& points,
			double epsilon,
			unsigned int minpts);

private:
	DBSCAN::Node* create_node(unsigned int intdex);
	int append_at_end(unsigned int index, EpsilonNeighbours *en);

	DBSCAN::EpsilonNeighbours* get_epsilon_neighbours(
			unsigned int index,
			const std::vector<Point3*>& points,
			double epsilon);
	void destroy_epsilon_neighbours(EpsilonNeighbours *en);
	int expand(
			std::vector<DBSCAN::Cluster>& clusters,
			unsigned int index,
			unsigned int cluster_id,
			const std::vector<Point3*>& points,
			double epsilon,
			unsigned int minpts);
	int spread(
			std::vector<DBSCAN::Cluster>& clusters,
			unsigned int index,
			EpsilonNeighbours *seeds,
			unsigned int cluster_id,
			const std::vector<Point3*>& points,
			double epsilon,
			unsigned int minpts);
	double euclidean_dist(Point3* a, Point3* b);
	double adjacent_intensity_dist(Point3* a, Point3* b);
};
