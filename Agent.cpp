#include "Agent.h"

Agent::Agent()
{
	start = glm::vec3(0.0f, 0.0f, 0.0f);
	goal = glm::vec3(0.0f, 0.0f, 0.0f);
	velocity = glm::vec3(0.0f, 0.0f, 0.0f);
	current_pos = 1;
	size = 0.5f;
	color = glm::vec3(0.0f, 0.0f, 1.0f);
	//gotpath = false;
}

bool Agent::CollisionTestObstacle(Obstacle* obs, int obs_num) {
	for (int i = 0; i < obs_num; i++) {
		if (obs[i].CollisionDetect(position, size)) {
			return true;
		}
	}
	return false;
}

bool Agent::CollisionTestAgent(Agent agent) {
	float distance = glm::distance(agent.position, position);
	return(distance <= agent.size + size);
}

bool Agent::IntersectionTestAgent(Agent agent) {
	float distance = glm::distance(agent.position, position);
	return(distance <= agent.size + size);
}

void Agent::SetPath() {
	//for()
}

void Agent::PrintPath(vector <glm::vec3> node) {
	printf("path:\n");
	for (int i = 0; i < path.size() - 1; i++) {
		int idx = path[i];
		printf("(%f %f %f) ->\n", node[idx].x, node[idx].y, node[idx].z);
	}
	int end = path.size() - 1;
	printf("(%f %f %f) ->\n", node[path[end]].x, node[path[end]].y, node[path[end]].z);
}

void Agent::PrintPathRRT() {
	printf("path:\n");
	for (int i = 0; i < pathrrt.size() - 1; i++) {
		printf("(%f %f %f) ->\n", pathrrt[i].x, pathrrt[i].y, pathrrt[i].z);
	}
	int i = pathrrt.size() - 1;
	printf("(%f %f %f) ->\n", pathrrt[i].x, pathrrt[i].y, pathrrt[i].z);
}

float Agent::distance(Agent a) {
	return(glm::distance(position, a.position));
}