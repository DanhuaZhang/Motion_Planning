#pragma once
#include "Obstacle.h"

Obstacle::Obstacle() {
	position = glm::vec3(0, 0, 0);
	radius = 1.0f;
}

Obstacle::Obstacle(glm::vec3 p, float r) {
	position = p;
	radius = r;
}

void Obstacle::SetPosition(glm::vec3 p) {
	position = p;
}

void Obstacle::SetRadius(float r) {
	radius = r;
}

glm::vec3 Obstacle::GetPosition() {
	return position;
}

float Obstacle::GetRadius(){
	return radius;
}

bool Obstacle::EdgeDetect(glm::vec3 p1, glm::vec3 p2, float size) {
	float A = p2.y - p1.y;
	float B = -(p2.x - p1.x);
	float C = -A * p1.x - B * p1.y;
	float distance = fabs(A*position.x + B * position.y + C) / sqrt(A*A + B * B);
	return(distance <= radius + size);
}

float Obstacle::DistanceToLine(glm::vec3 p1, glm::vec3 p2) {
	float A = p2.y - p1.y;
	float B = -(p2.x - p1.x);
	float C = -A * p1.x - B * p1.y;
	float distance = fabs(A*position.x + B * position.y + C) / sqrt(A*A + B * B);
	return distance;
}

bool Obstacle::CollisionDetect(glm::vec3 p, float size) {
	float distance = glm::distance(p, position);
	return(distance <= radius + size);
}