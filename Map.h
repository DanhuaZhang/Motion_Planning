#pragma once
#include "Graph.h"
#include "head.h"
#include "Agent.h"

class Map {
public:
	vector<Obstacle>obstacles;
	vector<Agent>agent; //use circle as agents, thus the only parameter is radius
	float length, width, height = 0.0f;
	Graph graph;
	int obstacle_num;
	int agent_num;
	float agent_size;

	Map(string filename);
	void GeneratePoint(int n);
	bool FindAllPath(bool heuristic);
	void PrintAllPath();
	void PrintAllPathRRT();
	void GenerateGraph(int n);
	void UpdateAgentPosition(float dt);
	void UpdateAgentPosition_Smooth(float dt);
	void UpdateAgentPosition_Boid(float dt, vector<glm::vec3>node);
	void UpdateAgentPosition_TTC(float dt, vector<glm::vec3>node);
	void GeneratePathRRT();
	void UpdateAgentPositionRRT(float dt);
};

float randf();
