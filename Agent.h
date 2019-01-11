#pragma once
#include "head.h"
#include "Obstacle.h"

class Agent
{
public:
	float size;
	glm::vec3 position;
	glm::vec3 velocity;
	glm::vec3 acceleration;
	glm::vec3 force;
	glm::vec3 goal_velocity;
	glm::vec3 start;// = glm::vec3(0.0f, 0.0f, 0.0f);
	glm::vec3 goal;// = glm::vec3(0.0f, 0.0f, 0.0f);
	glm::vec3 color;
	bool gotpath;
	vector<int>path;
	vector<glm::vec3>pathrrt;
	vector<glm::vec3>noderrt;
	vector <vector <float>> arcrrt;
	int current_pos; // index in the path
	int furthest_idx; // for smooth path

	Agent();
	float distance(Agent a);
	bool CollisionTestObstacle(Obstacle* obs, int obs_num);
	bool CollisionTestAgent(Agent agent);
	bool IntersectionTestAgent(Agent agent);
	void SetPath();
	void PrintPath(vector <glm::vec3> node);
	void PrintPathRRT();
};

