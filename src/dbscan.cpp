#include "hands_3d/dbscan.h"

DBSCAN::Node* DBSCAN::create_node(unsigned int index)
{
	DBSCAN::Node *n = new DBSCAN::Node();
	if (n == NULL) {
		return NULL;
	} else {
		n->index = index;
		n->next = NULL;
	}
	return n;
}

int DBSCAN::append_at_end(unsigned int index, DBSCAN::EpsilonNeighbours *en)
{
	DBSCAN::Node *n = create_node(index);
	if (n == NULL) {
		delete en;
		return FAILURE;
	}
	if (en->head == NULL) {
		en->head = n;
		en->tail = n;
	} else {
		en->tail->next = n;
		en->tail = n;
	}
	++(en->num_members);
	return SUCCESS;
}

//returns the epsilon neighbours of a data point
DBSCAN::EpsilonNeighbours* DBSCAN::get_epsilon_neighbours(unsigned int index, const std::vector<Point3*>& points, double epsilon)
{
	DBSCAN::EpsilonNeighbours *en = new DBSCAN::EpsilonNeighbours();
	if (en == NULL) {
		return en;
	}
	for (int i = 0; i < points.size(); ++i) {
		if (i == index) {
			continue;
		}
		if (euclidean_dist(points[index], points[i]) > epsilon) {
			continue;
		} else {
			if (append_at_end(i, en) == FAILURE) {
				destroy_epsilon_neighbours(en);
				en = NULL;
				break;
			}
		}
	}
	return en;
}

//Destroying epsilon neighbours
void DBSCAN::destroy_epsilon_neighbours(DBSCAN::EpsilonNeighbours *en)
{
	if (en) {
		Node *t, *h = en->head;
		while (h) {
			t = h->next;
			delete h;
			h = t;
		}
		delete en;
	}
}

//DBSCAN Algorithm
std::vector<DBSCAN::Cluster> DBSCAN::cluster(const std::vector<Point3*>& points, double epsilon, unsigned int minpts)
{
	std::vector<DBSCAN::Cluster> clusters;
	clusters.resize(1);
	clusters[0].cluster_id = 0;
	unsigned int i, cluster_id = 0;
	for (i = 0; i < points.size(); ++i) {
		if (points[i]->cluster_id == UNCLASSIFIED) {
			if (expand(clusters, i, cluster_id, points, epsilon, minpts) == CORE_POINT) {
				++cluster_id;
				clusters.resize(cluster_id+1);
				clusters[cluster_id].cluster_id = cluster_id;
			}
		}
	}
	return clusters;
}

//Determining if a new cluster can be created starting with one data point
int DBSCAN::expand(std::vector<DBSCAN::Cluster>& clusters, unsigned int index, unsigned int cluster_id, const std::vector<Point3*>& points, double epsilon, unsigned int minpts)
{
	int return_value = NOT_CORE_POINT;
	DBSCAN::EpsilonNeighbours *seeds = get_epsilon_neighbours(index, points, epsilon);
	if (seeds == NULL) {
		return FAILURE;
	}

	if (seeds->num_members < minpts) {
		points[index]->cluster_id = NOISE;
	} else {
		points[index]->cluster_id = cluster_id;
		clusters[cluster_id].points.push_back(points[index]);
		//Add epsilon neighbours to the same cluster
		Node *h = seeds->head;
		while (h) {
			points[h->index]->cluster_id = cluster_id;
			clusters[cluster_id].points.push_back(points[h->index]);
			h = h->next;
		}

		h = seeds->head;

		//See how far the cluster spreads
		while (h) {
			spread(clusters, h->index, seeds, cluster_id, points, epsilon, minpts);
			h = h->next;
		}

		return_value = CORE_POINT;
	}
	destroy_epsilon_neighbours(seeds);
	return return_value;
}

int DBSCAN::spread(std::vector<Cluster>& clusters, unsigned int index, DBSCAN::EpsilonNeighbours *seeds, unsigned int cluster_id, const std::vector<Point3*>& points, double epsilon, unsigned int minpts)
{
	DBSCAN::EpsilonNeighbours *spread = get_epsilon_neighbours(index, points, epsilon);
	if (spread == NULL) {
		return FAILURE;
	}
	//Process epsilon neighbours of neighbour
	if (spread->num_members >= minpts) {
		Node *n = spread->head;
		Point3 *d;
		while (n) {
			d = points[n->index];
			if (d->cluster_id == NOISE || d->cluster_id == UNCLASSIFIED) {
				if (d->cluster_id == UNCLASSIFIED) {
					if (append_at_end(n->index, seeds) == FAILURE) {
						destroy_epsilon_neighbours(spread);
						return FAILURE;
					}
				}
				d->cluster_id = cluster_id;
				clusters[cluster_id].points.push_back(d);
			}
			n = n->next;
		}
	}

	destroy_epsilon_neighbours(spread);
	return SUCCESS;
}

double DBSCAN::euclidean_dist(Point3* a, Point3* b)
{
	return sqrt(pow(a->x - b->x, 2) +
			pow(a->y - b->y, 2) +
			pow(a->z - b->z, 2));
}
