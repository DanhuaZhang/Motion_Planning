#pragma once
#include "Graph.h"

Graph::Graph()
{
}

void Graph::SetMaxEdgeLength(float m) {
	max_edge_length = m;
}

void Graph::CreateGraph(glm::vec3 start, glm::vec3 goal, vector<Obstacle>obstacles) {
	int num = node.size();
	arc.resize(num + 2);
	for (int i = 0; i < num + 2; i++) {
		arc[i].resize(num + 2);
	}
	arc[num][num] = 0.0f;
	arc[num+1][num+1] = 0.0f;
	bool notconnect = false;
	for (int i = 0; i < obstacles.size(); i++) {
		notconnect = obstacles[i].EdgeDetect(start, goal, 0.5);
		if (notconnect) {
			break;
		}
	}
	if (notconnect) {
		arc[num][num + 1] = 0.0f;
	}
	else {
		arc[num][num + 1] = glm::distance(start, goal);
	}
	arc[num+1][num] = arc[num][num+1];

	node.push_back(start);
	node.push_back(goal);
	//PrintEdge();
}

bool Graph::FindPath(int start, glm::vec3 goal, bool heuristic) {
	// use Uniform Cost Search algorithm for search the shortest path
	if (!path.empty()) {
		path.clear();
	}
	if (heuristic) { // A* Search
		vector <Dist> fn;
		for (int i = 0; i < node.size(); i++) {
			Dist d;
			d.cost = 0.0f;
			d.index = i;
			d.checked = false;
			d.heuris = glm::distance(node[i], goal);
			fn.push_back(d);
		}

		vector <Dist> passed_path;
		priority_queue <Dist, vector<Dist>, cmp_heuristic> priority;
		fn[start].parent = -1;
		priority.push(fn[start]);

		while (!priority.empty()) {
			int top = priority.top().index;
			if (fn[top].checked) {
				priority.pop();
				continue;
			}
			fn[top].checked = true;
			//printf("\npassed node:\n%d (%f, %f, %f)\n", top, node[top].x, node[top].y, node[top].z);

			passed_path.push_back(priority.top());
			//passed_path.push_back(gn[top]);
			priority.pop();

			if (node[top].x == goal.x && node[top].y == goal.y && node[top].z == goal.z) {
				//printf("putting node:\n");
				int i = passed_path.size() - 1; // get entries in passed_path
				int idx = passed_path[i].index; // get entries in node
				while (i > 0) {
					path.push_back(idx);
					//printf("%d (%f, %f, %f)\n", idx, node[idx].x, node[idx].y, node[idx].z);				
					idx = passed_path[i].parent;
					for (int j = 0; j <= i; j++) {
						if (passed_path[j].index == idx) {
							i = j;
							break;
						}
					}
				}
				path.push_back(start);
				reverse(path.begin(), path.end());
				return true;
			}
			else {
				for (int i = 0; i < node.size(); i++) {
					if (arc[top][i] != 0 && fn[i].checked == false) {
						if (fn[i].cost == 0 || fn[i].cost > fn[top].cost + arc[top][i]) {
							fn[i].cost = fn[top].cost + arc[top][i];
							fn[i].parent = top;
							priority.push(fn[i]);
						}
					}
				}
			}
		}

		return false;
	}
	else { // Uniform Cost Search
		vector <Dist> gn;
		for (int i = 0; i < node.size(); i++) {
			Dist d;
			d.cost = 0.0f;
			d.index = i;
			d.checked = false;
			gn.push_back(d);
		}

		vector <Dist> passed_path;
		priority_queue <Dist, vector<Dist>, cmp> priority;
		gn[start].parent = -1;
		priority.push(gn[start]);

		while (!priority.empty()) {
			int top = priority.top().index;
			if (gn[top].checked) {
				priority.pop();
				continue;
			}
			gn[top].checked = true;
			//printf("\npassed node:\n%d (%f, %f, %f)\n", top, node[top].x, node[top].y, node[top].z);

			passed_path.push_back(priority.top());
			//passed_path.push_back(gn[top]);
			priority.pop();

			if (node[top].x == goal.x && node[top].y == goal.y && node[top].z == goal.z) {
				//get path
				//printf("passed path:\n");
				//for (int i = 0; i < passed_path.size(); i++) {
				//	int idx = passed_path[i].index;
				//	printf("%d (%f, %f, %f)\t", idx, node[idx].x, node[idx].y, node[idx].z);
				//	idx = passed_path[i].parent;
				//	printf("its parent %d (%f, %f, %f)\n\n",idx, node[idx].x, node[idx].y, node[idx].z);
				//}

				//printf("putting node:\n");
				int i = passed_path.size() - 1; // get entries in passed_path
				int idx = passed_path[i].index; // get entries in node
				while (i > 0) {
					path.push_back(idx);
					//printf("%d (%f, %f, %f)\n", idx, node[idx].x, node[idx].y, node[idx].z);				
					idx = passed_path[i].parent;
					for (int j = 0; j <= i; j++) {
						if (passed_path[j].index == idx) {
							i = j;
							break;
						}
					}
				}
				path.push_back(start);
				reverse(path.begin(), path.end());
				return true;
			}
			else {
				for (int i = 0; i < node.size(); i++) {
					if (arc[top][i] != 0 && gn[i].checked == false) {
						if (gn[i].cost == 0 || gn[i].cost > gn[top].cost + arc[top][i]) {
							gn[i].cost = gn[top].cost + arc[top][i];
							gn[i].parent = top;
							priority.push(gn[i]);
						}

					}
				}
			}

			//printf("modified gn:\n");
			//for (int i = 0; i < gn.size(); i++) {
			//	printf("[%d %f]\t", i, gn[i].cost);
			//}

		}

		return false;
	}
}

void Graph::DeleteNode(int i) {
	node.erase(node.begin() + i);
	arc.erase(arc.begin() + i);
	for (int j = 0; j < arc.size(); j++) {
		arc[j].erase(arc[j].begin() + i);
	}
}

void Graph::InsertNode(glm::vec3 n) {
	node.push_back(n);

	int num = node.size();
	arc.resize(num);
	for (int i = 0; i < num; i++) {
		arc[i].resize(num);
	}

	int pre_num = num - 1;
	for (int j = 0; j < num; j++) {
		arc[pre_num][j] = 0.0f;
	}

	for (int i = 0; i < pre_num; i++) {
			arc[i][pre_num] = 0.0f;
	}
}

void Graph::PrintEdge() {
	printf("Generated edges:\n");
	for (int i = 0; i < arc.size(); i++) {
		for (int j = 0; j < arc.size(); j++) {
			printf("%f\t", arc[i][j]);
		}
		printf("\n");
	}
}

void Graph::PrintVertices() {
	printf("Generated vertices:\n");
	for (int i = 0; i < node.size(); i++) {
		printf("vertex[%d]: %f %f %f\n", i, node[i].x, node[i].y, node[i].z);
	}
}

void Graph::PrintPath() {
	printf("path:\n");
	for (int i = 0; i < path.size()-1; i++) {
		int idx = path[i];
		printf("(%f %f %f) ->\n", node[idx].x, node[idx].y, node[idx].z);
	}
	int end = path.size() - 1;
	printf("(%f %f %f) ->\n", node[path[end]].x, node[path[end]].y, node[path[end]].z);
}