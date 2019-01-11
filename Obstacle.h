#pragma once
#include "head.h"

class Obstacle
{
private:
	glm::vec3 position;
	float radius;

public:
	Obstacle();
	Obstacle(glm::vec3 p, float r);
	void SetPosition(glm::vec3 p);
	void SetRadius(float r);
	glm::vec3 GetPosition();
	float GetRadius();
	//check the edge between p1 and p2, true if collision is detected
	bool EdgeDetect(glm::vec3 p1, glm::vec3 p2, float size);
	//check if the point is in the disk, true if in
	bool CollisionDetect(glm::vec3 p, float size);
	float DistanceToLine(glm::vec3 p1, glm::vec3 p2);
};

