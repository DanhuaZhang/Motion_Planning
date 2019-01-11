#pragma once
#include "Map.h"

Map::Map(string filename) {
	agent_num = 0;
	obstacle_num = 0;
	FILE *fp;
	long file_length;
	char line[MAX_CHARACTER];
	// open the file containing the scene description
	fp = fopen(filename.c_str(), "r");

	// check for errors in opening the file
	if (fp == NULL) {
		printf("Can't open file '%s'\n", filename.c_str());
		exit(0);  //Exit
	}
	// determine the file size (this is optional -- feel free to delete the 4 lines below)
	fseek(fp, 0, SEEK_END); // move position indicator to the end of the file;
	file_length = ftell(fp);  // return the value of the current position
	printf("File '%s' is %ld bytes long.\n\n", filename.c_str(), file_length);
	fseek(fp, 0, SEEK_SET);  // move position indicator to the start of the file

	int current_obs = 0;
	int current_agent = 0;
	while (fgets(line, MAX_CHARACTER, fp)) { //Assumes no line is longer than 1024 characters!
		char command[100];
		int fieldsRead = sscanf(line, "%s ", command); //Read first word in the line (i.e., the command type)

		if (fieldsRead < 1) { //No command read
							  //Blank line
			continue;
		}

		if (strcmp(command, "length") == 0) {
			sscanf(line, "length %f", &length);
			printf("length:\t%f\n", length);
		}
		else if (strcmp(command, "width") == 0) {
			sscanf(line, "width %f", &width);
			printf("width:\t%f\n", width);
		}	
		else if (strcmp(command, "agent_number") == 0) {
			sscanf(line, "agent_number  %d", &agent_num);
			printf("\nagent_number:\t%d\n", agent_num);
			agent.resize(agent_num);
		}
		else if (strcmp(command, "agent_size") == 0) {
			sscanf(line, "agent_size  %f", &agent[current_agent].size);
			printf("agent_size:\t%f", agent[current_agent].size);
		}
		else if (strcmp(command, "start") == 0) {
			sscanf(line, "start %f %f", &agent[current_agent].start.x, &agent[current_agent].start.y);
			printf("\nstart:\t(%f, %f, %f)\n", agent[current_agent].start.x, agent[current_agent].start.y, agent[current_agent].start.z);
			agent[current_agent].position = agent[current_agent].start;
			agent[current_agent].current_pos = 1;
		}
		else if (strcmp(command, "goal") == 0) {
			sscanf(line, "goal %f %f", &agent[current_agent].goal.x, &agent[current_agent].goal.y);
			printf("goal:\t(%f, %f, %f)\n\n", agent[current_agent].goal.x, agent[current_agent].goal.y, agent[current_agent].goal.z);
			current_agent++;
		}		
		else if (strcmp(command, "obstacle_number") == 0) {
			sscanf(line, "obstacle_number  %d", &obstacle_num);
			printf("\nobstacle_number:\t%d\n", obstacle_num);
			obstacles.resize(obstacle_num);
		}
		else if (strcmp(command, "position") == 0) {
			glm::vec3 p = glm::vec3(0.0f,0.0f,0.0f);
			sscanf(line, "position %f %f", &p.x, &p.y);
			obstacles[current_obs].SetPosition(p);
			printf("position:\t(%f, %f)\n", obstacles[current_obs].GetPosition().x, obstacles[current_obs].GetPosition().y);		
		}
		else if (strcmp(command, "radius") == 0) {
			float r;
			sscanf(line, "radius %f", &r);
			obstacles[current_obs].SetRadius(r);
			printf("radius:\t%f\n\n", obstacles[current_obs].GetRadius());
			current_obs++;
		}
	}

	for (int i = 0; i < agent_num; i++) {
		graph.CreateGraph(agent[i].start, agent[i].goal,obstacles);
	}

	//graph.PrintVertices();
	//graph.PrintEdge();
}

float randf() {
	return (float)(rand() % 1000) * 0.001f;
}

void Map::GeneratePoint(int n) {
	int valid_point = 0;
	float x, y;
	//graph.PrintEdge();
	int pre_num = graph.node.size();

	// generate new vertices
	for (int i = 0; i < n; i++) {
		x = randf() * length - length / 2;
		y = randf() * width - width / 2;
		graph.node.push_back(glm::vec3(x, y, 0.0f));
	}
	
	//realloc memory for arc
	int num = graph.node.size();
	graph.arc.resize(num);
	for (int i = 0; i < num; i++) {
		graph.arc[i].resize(num);
	}

	for (int i = pre_num; i < num; i++) {
		for (int j = 0; j < num; j++) {
			graph.arc[i][j] = 0.0f;
		}
	}
	for (int i = 0; i < pre_num; i++) {
		for (int j = pre_num; j < num; j++) {
			graph.arc[i][j] = 0.0f;
		}
	}

	// for each new vertex, build distance
	int i = 2*agent_num;
	while (i < graph.node.size()) {
		//printf("i %d\n", i);
		//if (i == 21)system("pause");
		//printf("vertex[%d]: %f %f %f\n", i, graph.node[i].x, graph.node[i].y, graph.node[i].z);
		
		bool abandon = false;
		bool abandon_edge = false;

		//check if in the obstacle
		// if new-generated vertex is in the disk, then abandon
		for (int j = 0; j < obstacle_num; j++) {
			abandon = obstacles[j].CollisionDetect(graph.node[i], agent_size);
			if (abandon) {
				graph.DeleteNode(i);
				break;
			}
		}
		if (abandon)continue;

		// for each nearby existing vertex, check edge collision
		// if intersected, then abandon
		for (int j = 0; j!=i && j < graph.node.size(); j++) {
			float distance = glm::distance(graph.node[i], graph.node[j]);
			// if overlap the existing vertices, then abandon
			/*if (distance < graph.min_edge_length) {
				abandon = true;
				graph.DeleteNode(i);
				break;
			}*/

			// if in the nearby range
			/*if (distance < graph.max_edge_length) {
				for (int k = 0; k < obstacle_num; k++) {
					abandon_edge = obstacles[k].EdgeDetect(graph.node[i],graph.node[j],agent_size + 0.05);
					if (abandon_edge)break;
				}
				if (!abandon_edge) {
					graph.arc[i][j] = distance;
					graph.arc[j][i] = distance;
				}
			}*/
			for (int k = 0; k < obstacle_num; k++) {
				abandon_edge = obstacles[k].EdgeDetect(graph.node[i], graph.node[j], 0.5);
				if (abandon_edge)break;
			}
			if (!abandon_edge) {
				graph.arc[i][j] = distance;
				graph.arc[j][i] = distance;
			}
		}

		if (!abandon) {
			i++;
		}
	}

	//printf("new edges:\n");
	//printf("rows:\n");
	//for (int i = pre_num; i < graph.arc.size(); i++) {
	//	for (int j = 0; j < graph.arc.size(); j++) {
	//		printf("%f\t", graph.arc[i][j]);
	//	}
	//	printf("\n");
	//}
	//printf("columns:\n");
	//for (int i = 0; i < pre_num; i++) {
	//	for (int j = pre_num; j < graph.arc.size(); j++) {
	//		printf("%f\t", graph.arc[i][j]);
	//	}
	//	printf("\n");
	//}

	//graph.PrintVertices();
	//graph.PrintEdge();
}

bool Map::FindAllPath(bool heuristic) {
	bool findpath;
	for (int i = 0; i < agent_num; i++) {
		if (!agent[i].path.empty()) {
			agent[i].path.clear();
		}
		findpath = graph.FindPath(i * 2, agent[i].goal, heuristic);
		if (findpath) {
			agent[i].path = graph.path;
			agent[i].gotpath = true;
		}
		else {
			return false;
		}	
	}
}

void Map::PrintAllPath() {
	for (int i = 0; i < agent_num; i++) {
		printf("Agent No.%d\n", i);
		agent[i].PrintPath(graph.node);
	}
}

void Map::PrintAllPathRRT() {
	for (int i = 0; i < agent_num; i++) {
		printf("Agent No.%d\n", i);
		agent[i].PrintPathRRT();
	}
}

void Map::GenerateGraph(int n) {
	float x, y;
	
	int pre_num = graph.node.size();

	// generate new vertices
	graph.node.push_back(glm::vec3(-3.0f, 0.0f, 0.0f));
	graph.node.push_back(glm::vec3(9.0f, 0.0f, 0.0f));
	graph.node.push_back(glm::vec3(0.0f, 3.0f, 0.0f));

	//realloc memory for arc
	int num = graph.node.size();
	graph.arc.resize(num);
	for (int i = 0; i < num; i++) {
		graph.arc[i].resize(num);
	}
	for (int i = pre_num; i < num; i++) {
		for (int j = 0; j < num; j++) {
			graph.arc[i][j] = 0.0f;
		}
	}
	for (int i = 0; i < pre_num; i++) {
		for (int j = pre_num; j < num; j++) {
			graph.arc[i][j] = 0.0f;
		}
	}

	graph.arc[0][2] = glm::distance(graph.node[0],graph.node[2]);
	graph.arc[2][0] = graph.arc[0][2];

	graph.arc[0][3] = glm::distance(graph.node[0], graph.node[3]);
	graph.arc[3][0] = graph.arc[0][3];

	graph.arc[1][3] = glm::distance(graph.node[1], graph.node[3]);
	graph.arc[3][1] = graph.arc[1][3];

	graph.arc[2][3] = glm::distance(graph.node[3], graph.node[2]);
	graph.arc[3][2] = graph.arc[2][3];

	graph.arc[2][4] = glm::distance(graph.node[2], graph.node[4]);
	graph.arc[4][2] = graph.arc[2][4];

	graph.arc[1][4] = glm::distance(graph.node[1], graph.node[4]);
	graph.arc[4][1] = graph.arc[1][4];

	graph.arc[3][4] = glm::distance(graph.node[3], graph.node[4]);
	graph.arc[4][3] = graph.arc[3][4];

	graph.PrintVertices();
	graph.PrintEdge();
}

void Map:: UpdateAgentPosition(float dt) {
	for (int i = 0; i < agent_num; i++) {
		if (agent[i].current_pos < agent[i].path.size()) {
			agent[i].velocity = glm::normalize(graph.node[agent[i].path[agent[i].current_pos]] - graph.node[agent[i].path[agent[i].current_pos - 1]]);
			agent[i].position = agent[i].position + agent[i].velocity * dt;//*2.0f;
			float dis1 = glm::distance(agent[i].position, graph.node[agent[i].path[agent[i].current_pos - 1]]);
			float dis2 = glm::distance(graph.node[agent[i].path[agent[i].current_pos]], graph.node[agent[i].path[agent[i].current_pos - 1]]);
			if (dis1 > dis2) {
				agent[i].position = graph.node[agent[i].path[agent[i].current_pos]];
				agent[i].current_pos++;
			}
		}
		
	}
}

void Map::UpdateAgentPosition_Smooth(float dt) {
	for (int i = 0; i < agent_num; i++) {
		// find the furthest visible vertex in the path				
		bool collision = false;
		int new_pos = -1;
		//int k = agent[i].current_pos;

		//int p = agent[i].path[0];
		//int q = agent[i].path[1];
		//for (int k = 0; k < obstacle_num; k++) {
		//	collision = obstacles[k].EdgeDetect(graph.node[p], graph.node[q], 0.5);
		//	if (collision)break;
		//}
		//collision = obstacles[0].EdgeDetect(graph.node[p], graph.node[q], 0.5);
		////collision = obstacles[0].EdgeDetect(graph.node[agent[i].path[0]], graph.node[agent[i].path[1]], agent[i].size);
		//if (collision) {
		//	printf("obs: %d\t collision: true\n", 0, collision);
		//}
		//else {
		//	printf("obs: %d\t collision: false\n", 0, collision);
		//}
		//
		//printf("vertex[%d]: (%f,%f,%f)\n", p, graph.node[p].x, graph.node[p].y, graph.node[p].z);
		//printf("vertex[%d]: (%f,%f,%f)\n\n\n\n", q, graph.node[q].x, graph.node[q].y, graph.node[q].z);
		//("arc: %f\n", graph.arc[agent[i].path[k]][0]);
		for (int k = agent[i].path.size() - 1; k >= agent[i].current_pos; k--) {
			int obs_checked = 0;
			for (int j = 0; j < obstacle_num; j++) {
				collision = obstacles[j].EdgeDetect(agent[i].position, graph.node[agent[i].path[k]], 0.5);
				if (!collision) {
					obs_checked++;
				}
				//printf("obs: %d\t collision: %d\n", j, collision);
			}
			if (obs_checked == obstacle_num) {
				new_pos = k;
				//printf("\nhhh\n");
				break;
			}
		}
		if (new_pos == -1) { //get stuck
			//replan
			printf("\n\nwoo\n");
			/*graph.InsertNode(agent[i].position);
			bool findpath = false;
			while (!findpath) {
				findpath = graph.FindPath(graph.node.size() - 1, agent[i].goal, true);
				GeneratePoint(20);
			}			
			agent[i].path = graph.path;
			agent[i].current_pos = 1;*/
		}
		else {
			agent[i].current_pos = new_pos;
		}
		
		// calculate the direction
		glm::vec3 prefernode = graph.node[agent[i].path[agent[i].current_pos]];
		agent[i].velocity = glm::normalize(prefernode - agent[i].position);
		agent[i].position = agent[i].position + agent[i].velocity *dt;		

		//printf("i:%d currentpos: %d new_pos: %d\nvelocity: (%f,%f,%f) position: (%f,%f,%f) prefernode: (%f,%f,%f)\n", i, agent[i].current_pos, new_pos,\
		//			agent[i].velocity.x, agent[i].velocity.y, agent[i].velocity.z, \
		//	agent[i].position.x, agent[i].position.y, agent[i].position.z,\
		//	prefernode.x, prefernode.y, prefernode.z);

	}
}

void Map::UpdateAgentPosition_Boid(float dt, vector<glm::vec3>node) {
	float neighbour = 3.0f;
	float max_force = 10.0f;
	glm::vec3 seperate, alignment, cohesion;

	// take obstacle into account
	vector<Agent>new_agent;
	for (int i = 0; i < obstacle_num; i++) {
		Agent ag;
		ag.position = obstacles[i].GetPosition();
		ag.current_pos = -1;
		ag.size = obstacles[i].GetRadius();
		new_agent.push_back(ag);
	}

	for (int i = 0; i < agent_num; i++) {
		seperate = glm::vec3(0.0f);
		alignment = agent[i].velocity;
		cohesion = glm::vec3(0.0f);
		int nei_num = 0;
		int align_num = 0;
		for (int j = 0; j!=i &&j < agent_num; j++) {
			//get agent[i]'s neighbor
			float d = agent[i].distance(agent[j]);
			if (d <= neighbour) {
				seperate += 1 / d /d;// / d;
				alignment += agent[j].velocity;
				cohesion += agent[j].position;
				nei_num++;
			}
		}
		if (nei_num != 0) {
			align_num = nei_num;
			for (int j = 0; j < new_agent.size(); j++) {
				//get agent[i]'s neighbor
				float d = agent[i].distance(new_agent[j]);
				if (d <= neighbour) {
					seperate += 1 / d / d;// / d;
					align_num++;
				}
			}
		}
		
		if (nei_num != 0) {
			seperate = seperate / (float)nei_num;
			alignment = alignment / (float)(align_num +1);
			cohesion = cohesion / (float)nei_num - agent[i].position;
		}
		/*agent[i].force = seperate + 1.4f*(alignment - agent[i].velocity) + 0.5f* cohesion;*/
		agent[i].force = seperate + alignment + cohesion;
		if (glm::length(agent[i].force) > max_force) {
			agent[i].force = glm::normalize(agent[i].force)*max_force;
		}
		
		//printf("force: (%f,%f,%f)\n", agent[i].force.x, agent[i].force.y, agent[i].force.z);

		//furthest visible path node for each agent

		bool collision = false;
		int new_pos = -1;
		for (int k = agent[i].path.size() - 1; k >= agent[i].current_pos; k--) {
			int obs_checked = 0;
			for (int j = 0; j < obstacle_num; j++) {
				collision = obstacles[j].EdgeDetect(agent[i].position, graph.node[agent[i].path[k]], 0.5);
				if (!collision) {
					obs_checked++;
				}
				//printf("obs: %d\t collision: %d\n", j, collision);
			}
			if (obs_checked == obstacle_num) {
				new_pos = k;
				//printf("\nhhh\n");
				break;
			}
		}
		if (new_pos == -1) { //get stuck
			//replan
			printf("\n\nwoo\n");
			//printf("agent[%d].path:\n", i);
			//graph.InsertNode(agent[i].position);
			//int new_start = graph.node.size() - 1;
			//bool findpath = false;
			//while (!findpath) {
			//	GeneratePoint(20);
			//	printf("nodesize: %d\n", graph.node.size());
			//	findpath = graph.FindPath(new_start, agent[i].goal, false);			
			//	printf("tried\n");
			//}
			//printf("newpath!!!!!!\n");
			//agent[i].path = graph.path;
			//for (int j = 0; j < agent[i].path.size(); j++) {
			//	
			//	int idx = agent[i].path[j];
			//	printf("path[%d]: (%f,%f,%f) ->\n", j, graph.node[idx].x, graph.node[idx].y, graph.node[idx].z);
			//}
			//agent[i].current_pos = 1;
			
		}
		else {
			agent[i].current_pos = new_pos;
		}


		//bool collision = false;
		//for (int k = agent[i].current_pos + 1; k < agent[i].path.size(); k++) {
		//	for (int j = 0; j < obstacle_num; j++) {
		//		collision = obstacles[j].EdgeDetect(agent[i].position, graph.node[agent[i].path[k]],0.5);
		//		if (collision) {
		//			break;
		//		}
		//	}
		//	if (collision) {
		//		agent[i].current_pos = k - 1;
		//		//printf("i: %d\tk: %d\tcurrent_pos: %d\n", i, k, agent[i].current_pos);
		//		break;
		//	}
		//}
		//if (!collision) {
		//	agent[i].current_pos = agent[i].path.size() - 1;
		//}
	}	

	//update here
	for (int i = 0; i < agent_num; i++) {	
		glm::vec3 prefernode = graph.node[agent[i].path[agent[i].current_pos]];
		//agent[i].goal_velocity = glm::normalize(prefernode - agent[i].position);
		agent[i].goal_velocity = prefernode - agent[i].position;
		agent[i].velocity = (agent[i].force + 0.2f* (glm::normalize(agent[i].goal_velocity - agent[i].velocity)));
		//agent[i].velocity = (agent[i].force + agent[i].goal_velocity);
		agent[i].position = agent[i].position + agent[i].velocity *dt;
		//agent[i].goal_velocity = glm::normalize(node[agent[i].path[agent[i].current_pos]] - agent[i].position);
		//agent[i].velocity = (agent[i].force + 1.4f*(agent[i].goal_velocity- agent[i].velocity))*dt;
		////agent[i].velocity = agent[i].force *0.1f*dt + agent[i].goal_velocity;
		//agent[i].position += agent[i].velocity*dt;
		
		//printf("velocity: (%f,%f,%f)\tposition: (%f,%f,%f)\n", \
		//	agent[i].velocity.x, agent[i].velocity.y, agent[i].velocity.z, \
		//	agent[i].position.x, agent[i].position.y, agent[i].position.z);
	}
}

void Map::UpdateAgentPosition_TTC(float dt, vector<glm::vec3>node) {
	float tH = 1.0;
	float maxF = 3.0;
	for (int i = 0; i < agent_num; i++) {	
		//furthest visible path node for each agent
		bool collision = false;
		for (int k = agent[i].current_pos + 1; k < agent[i].path.size(); k++) {
			for (int j = 0; j < obstacle_num; j++) {
				collision = obstacles[j].EdgeDetect(agent[i].position, graph.node[agent[i].path[k]], 0.5);
				if (collision) {
					break;
				}
			}
			if (collision) {
				agent[i].current_pos = k - 1;
				//printf("i: %d\tk: %d\tcurrent_pos: %d\n", i, k, agent[i].current_pos);
				break;
			}
		}
		if (!collision) {
			agent[i].current_pos = agent[i].path.size() - 1;
		}

		float tau;
		for (int j = 0; j < agent_num &&j != i; j++) {
			//compute tau
			float r = agent[i].size + agent[j].size;
			glm::vec3 w = agent[j].position - agent[i].position;
			float c = glm::dot(w, w) - r * r;
			if (c < 0)tau = 0;
			glm::vec3 v = agent[i].velocity - agent[j].velocity;
			float a = glm::dot(v, v);
			float b = glm::dot(v, w);
			float discr = b * b - a * c;
			if (discr <= 0)tau = INFINITE;
			tau = (b - sqrt(discr)) / a;
			if (tau < 0)tau = INFINITE;

			//compute collision avoidance force
			agent[i].force = (agent[i].position + agent[i].velocity*dt) - (agent[j].position + agent[j].velocity*dt);		
			
			if (glm::length(agent[i].force) != 0) {
				glm::normalize(agent[i].force);
			}

			float mag = 0;
			if (tau >= 0 && tau <= tH) {
				mag = (tH - tau) / (tau + 0.001);
			}
			if (mag > maxF) {
				mag = maxF;
			}
			agent[i].force *= mag;
			agent[i].force = agent[i].force;// +0.1f*(agent[i].goal_velocity - agent[i].velocity);
		}

		//update here
		for (int i = 0; i < agent_num; i++) {
			glm::vec3 prefernode = graph.node[agent[i].path[agent[i].current_pos]];
			//agent[i].goal_velocity = glm::normalize(prefernode - agent[i].position);
			agent[i].goal_velocity = prefernode - agent[i].position;
			agent[i].velocity = (agent[i].force + 0.5f*(glm::normalize(agent[i].goal_velocity - agent[i].velocity)));
			//agent[i].velocity = (agent[i].force + agent[i].goal_velocity);
			agent[i].position = agent[i].position + agent[i].velocity *dt;
			//agent[i].goal_velocity = glm::normalize(node[agent[i].path[agent[i].current_pos]] - agent[i].position);
			//agent[i].velocity = (agent[i].force + 1.4f*(agent[i].goal_velocity- agent[i].velocity))*dt;
			////agent[i].velocity = agent[i].force *0.1f*dt + agent[i].goal_velocity;
			//agent[i].position += agent[i].velocity*dt;

			printf("i:%d velocity: (%f,%f,%f)\tposition: (%f,%f,%f)\n", i, \
						agent[i].velocity.x, agent[i].velocity.y, agent[i].velocity.z, \
			agent[i].position.x, agent[i].position.y, agent[i].position.z);
		}
	}
}

void Map::GeneratePathRRT() {
	int n = 5;
	float x, y;
	for (int i = 0; i < agent_num; i++) {
		if (!agent[i].pathrrt.empty()) {
			agent[i].pathrrt.clear();
		}
		bool gotpath = false;	
		agent[i].noderrt.push_back(agent[i].start);
		agent[i].pathrrt.push_back(agent[i].start);
		while (!gotpath) {
			int pathend = agent[i].pathrrt.size() - 1;

			// generate a new vertice
			glm::vec3 newpoint;
			newpoint.x = agent[i].pathrrt[pathend].x + 3.0f*(randf() - 0.5f);
			newpoint.y = agent[i].pathrrt[pathend].y + 3.0f*(randf() - 0.5f);
			newpoint.z = 0.0f;
			//vector<glm::vec3>newpoints;
			//glm::vec3 newpoint;
			//for (int j = 0; j < n; j++) {			
			//	newpoint.x = agent[i].pathrrt[pathend].x + 3.0f*(randf() - 0.5f);
			//	newpoint.y = agent[i].pathrrt[pathend].y + 3.0f*(randf() - 0.5f);
			//	newpoint.z = 0.0f;
			//	newpoints.push_back(newpoint);
			//}

			////find the nearest node
			//float dis = INFINITY;
			//int nearest = 0;
			//for (int j = 0; j < n; j++) {
			//	float dist = glm::distance(newpoints[j], agent[i].pathrrt[pathend]);
			//	if (dist < dis) {
			//		dis = dist;
			//		nearest = j;
			//	}					
			//}
			//newpoint = newpoints[nearest];

			bool collision = false;
			//check if in the obstacle
			for (int j = 0; j < obstacle_num; j++) {
				collision = obstacles[j].CollisionDetect(newpoint, 0.5f);
				if (collision) {
					break;
				}
			}
			if (collision)continue;

			//check the last node in path with newpoint 
			collision = false;
			for (int j = 0; j < obstacle_num; j++) {
				collision = obstacles[j].EdgeDetect(agent[i].pathrrt[pathend], newpoint, 0.5f);
				if (collision) {
					break;
				}
			}
			//if the last node in the path and the new node cannot be connected, abandon the new node
			if (collision) {
				continue;
			}

			//check the goal state with the last node in path 
			collision = false;
			float maxdistance = 0.0f;
			int obs_idx = 0;
			for (int j = 0; j < obstacle_num; j++) {
				float d = obstacles[j].DistanceToLine(agent[i].goal, agent[i].pathrrt[pathend]);
				float r = obstacles[j].GetRadius();
				if (d <= r + 0.5f) {
					collision = true;
					//get the intersection node					
					float distance = sqrt(r*r - d * d);
					if (distance > maxdistance) {
						maxdistance = distance;
						obs_idx = j;
					}
				}
			}

			if (collision) {
				glm::vec3 dire = glm::normalize(agent[i].goal - agent[i].pathrrt[pathend]);
				glm::vec3 a = obstacles[obs_idx].GetPosition() - agent[i].pathrrt[pathend];
				float projection = glm::dot(a, dire);
				float mag = projection - maxdistance - 0.5;
				glm::vec3 node = agent[i].pathrrt[pathend] + dire * mag;
				agent[i].pathrrt.push_back(node);

			}
			else {
				agent[i].pathrrt.push_back(agent[i].goal);
				break;
			}

			agent[i].pathrrt.push_back(newpoint);	
			printf("pathsize: %d\n", agent[i].pathrrt.size());
		}			
	}	
}

void Map::UpdateAgentPositionRRT(float dt) {
	for (int i = 0; i < agent_num; i++) {
		if (agent[i].current_pos < agent[i].pathrrt.size()) {
			agent[i].velocity = glm::normalize(agent[i].pathrrt[agent[i].current_pos] - agent[i].pathrrt[agent[i].current_pos-1]);
			agent[i].position = agent[i].position + agent[i].velocity * dt;//*2.0f;
			float dis1 = glm::distance(agent[i].position, agent[i].pathrrt[agent[i].current_pos-1]);
			float dis2 = glm::distance(agent[i].pathrrt[agent[i].current_pos], agent[i].pathrrt[agent[i].current_pos-1]);
			if (dis1 > dis2) {
				agent[i].position = agent[i].pathrrt[agent[i].current_pos];
				agent[i].current_pos++;
			}
		}
		//printf("position: (%f,%f,%f)\n", agent[i].position.x, agent[i].position.y, agent[i].position.z);
	}
}