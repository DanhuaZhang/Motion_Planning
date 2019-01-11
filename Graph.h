#pragma once
#include "head.h"
#include "Obstacle.h"

class Graph {
public:
	vector <vector <float>> arc;
	vector <glm::vec3> node;
	//vector <glm::vec3> path;
	vector <int> path;
	float max_edge_length = 8.0f;
	float min_edge_length = 2.5f;

	Graph();
	void SetMaxEdgeLength(float m);
	void CreateGraph(glm::vec3 start, glm::vec3 goal, vector<Obstacle>obstacles);
	bool FindPath(int start, glm::vec3 goal, bool heuristic);
	void DeleteNode(int i);
	void PrintEdge();
	void PrintVertices();
	void PrintPath();
	void InsertNode(glm::vec3 n);
};

struct Dist {
	float cost;
	float heuris;
	int index;
	bool checked;
	int parent;
};

struct cmp {
	bool operator()(Dist a, Dist b) {
		return a.cost > b.cost;
	}
};

struct cmp_heuristic {
	bool operator()(Dist a, Dist b) {
		return (a.cost+a.heuris) > (b.cost+b.heuris);
	}
};