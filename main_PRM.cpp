#pragma once
#include "Graph.h"
#include "Map.h"
static char* readShaderSource(const char* shaderFile);
GLuint InitShader(const char* vShaderFileName, const char* fShaderFileName);
bool DEBUG_ON = true;
bool heuristic = true;

bool saveOutput = false; //Make to true to save out your animation 
int screen_width = 800;
int screen_height = 600;

bool fullscreen = false;
vector<int>passed;

// Sphere Shader sources
const GLchar* vertexSource =
"#version 150 core\n"
"in vec3 position;"
//the color will be changed in the OpenGL part
"uniform vec3 inColor;"
"in vec3 inNormal;"
"const vec3 inLightDir = normalize(vec3(0,2,2));"
"out vec3 Color;"
"out vec3 normal;"
"out vec3 lightDir;"
"uniform mat4 model;"
"uniform mat4 view;"
"uniform mat4 proj;"
"void main() {"
"   Color = inColor;"
"   gl_Position = proj * view * model * vec4(position, 1.0);"
"   vec4 norm4 = transpose(inverse(model)) * vec4(inNormal, 1.0);"
"   normal = normalize(norm4.xyz);"
"   lightDir = (view * vec4(inLightDir, 0)).xyz;"
"}";

const GLchar* fragmentSource =
"#version 150 core\n"
"in vec3 Color;"
"in vec3 normal;"
"in vec3 lightDir;"
"out vec4 outColor;"
"const float ambient = .2;"
"void main() {"
"   vec3 diffuseC = Color * max(dot(lightDir, normal), 0);"
"   vec3 ambC = Color * ambient;"
// the alpha channel of all pixels are 1.0, i.e. the opacity is 100%
"   outColor = vec4(diffuseC+ambC, 1.0);"
"}";

//Index of where to model, view, and projection matricies are stored on the GPU
GLint uniModel, uniView, uniProj, uniColor;

float aspect; //aspect ratio (needs to be updated if the window is resized)

int main(int argc, char *argv[]) {
	// generate the graph and get the path
	Map roadmap("local2.txt");
	bool allpathgot = false;
	int n = 20; // every time generate n new vertices
	//roadmap.GenerateGraph(n);
	//pathgot = roadmap.FindPath();
	//srand(2);
	srand(time(NULL));
	
	////generate new agents
	//int num_agent = 10;
	//for (int i = 0; i < num_agent; i++) {
	//	Agent agent;
	//	agent.start = glm::vec3(roadmap.agent[0].start.x-randf(), roadmap.agent[0].start.y-randf(), 0.0f);		
	//	agent.goal = glm::vec3(roadmap.agent[0].goal.x+randf(), roadmap.agent[0].goal.y+randf(), 0.0f);
	//	agent.position = agent.start;
	//	roadmap.agent.push_back(agent);
	//	roadmap.graph.CreateGraph(agent.start, agent.goal, roadmap.obstacles);
	//}
	//roadmap.agent_num = roadmap.agent.size();

	while (!allpathgot) {
		roadmap.GeneratePoint(n);
		allpathgot = roadmap.FindAllPath(heuristic);
	}
	//printf("Got all paths!\n");
	roadmap.PrintAllPath();

	SDL_Init(SDL_INIT_VIDEO);  //Initialize Graphics (for OpenGL)

	//Ask SDL to get a recent version of OpenGL (3.2 or greater)
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 4);

	//Create a window (offsetx, offsety, width, height, flags)
	SDL_Window* window = SDL_CreateWindow("My OpenGL Program", 100, 50, screen_width, screen_height, SDL_WINDOW_OPENGL);
	aspect = screen_width / (float)screen_height; //aspect ratio (needs to be updated if the window is resized)

	//Create a context to draw in
	SDL_GLContext context = SDL_GL_CreateContext(window);

	if (gladLoadGLLoader(SDL_GL_GetProcAddress)) {
		printf("\nOpenGL loaded\n");
		printf("Vendor:   %s\n", glGetString(GL_VENDOR));
		printf("Renderer: %s\n", glGetString(GL_RENDERER));
		printf("Version:  %s\n\n", glGetString(GL_VERSION));
	}
	else {
		printf("ERROR: Failed to initialize OpenGL context.\n");
		return -1;
	}

	//// Allocate Texture 0 (wood) ///////
	SDL_Surface* surface = SDL_LoadBMP("wood.bmp");
	if (surface == NULL) { //If it failed, print the error
		printf("Error: \"%s\"\n", SDL_GetError()); return 1;
	}
	GLuint tex0;
	glGenTextures(1, &tex0);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, tex0);

	//What to do outside 0-1 range
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	//Load the texture into memory
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, surface->w, surface->h, 0, GL_BGR, GL_UNSIGNED_BYTE, surface->pixels);
	glGenerateMipmap(GL_TEXTURE_2D);

	SDL_FreeSurface(surface);
	//// End Allocate Texture ///////

	//--------------------------------------
	//create cloth nodes (nv*nh)
	//the 0th row is the top row, fixed
	//--------------------------------------
	int cloth_numVerts = 6;
	int cloth_numLines = cloth_numVerts * 8;
	float cloth[6 * 8] = {
		//position, normal, texture_cord
		-1.0f,1.0f,0.0f, 0.0f,0.0f,1.0f, 0.0f,0.0f,
		-1.0f,-1.0f,0.0f, 0.0f,0.0f,1.0f, 0.0f,1.0f,
		1.0f,-1.0f,0.0f, 0.0f,0.0f,1.0f, 1.0f,1.0f,
		-1.0f,1.0f,0.0f, 0.0f,0.0f,1.0f, 0.0f,0.0f,
		1.0f,1.0f,0.0f, 0.0f,0.0f,1.0f, 1.0f,0.0f,
		1.0f,-1.0f,0.0f, 0.0f,0.0f,1.0f, 1.0f,1.0f
	};

	//--------------------------------------
	// load cloth
	//--------------------------------------
	GLuint vbo_cloth;
	glGenBuffers(1, &vbo_cloth);  //Create 1 buffer called vbo
	glBindBuffer(GL_ARRAY_BUFFER, vbo_cloth); //Set the vbo as the active array buffer (Only one buffer can be active at a time)
	glBufferData(GL_ARRAY_BUFFER, cloth_numLines * sizeof(float), cloth, GL_STATIC_DRAW);

	//Build a Vertex Array Object. This stores the VBO and attribute mappings in one object
	GLuint vao_cloth;
	glGenVertexArrays(1, &vao_cloth); //Create a VAO
	glBindVertexArray(vao_cloth); //Bind the above created VAO to the current context

	//Join the vertex and fragment shaders together into one program
	int ClothShaderProgram = InitShader("vertexTex.glsl", "fragmentTex.glsl");	
	glUseProgram(ClothShaderProgram);

	glBindVertexArray(vao_cloth);
	//Tell OpenGL how to set fragment shader input 
	GLint posAttrib = glGetAttribLocation(ClothShaderProgram, "position");
	glVertexAttribPointer(posAttrib, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), 0);
	//Attribute, vals/attrib., type, normalized?, stride, offset
	//Binds to VBO current GL_ARRAY_BUFFER 
	glEnableVertexAttribArray(posAttrib);

	GLint texAttrib = glGetAttribLocation(ClothShaderProgram, "inTexcoord");
	glEnableVertexAttribArray(texAttrib);
	glVertexAttribPointer(texAttrib, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));

	GLint nolAttrib = glGetAttribLocation(ClothShaderProgram, "inNormal");
	glVertexAttribPointer(nolAttrib, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(nolAttrib);

	glBindVertexArray(0); //Unbind the VAO

	//------------------------------
	//load sphere as obstacle
	//------------------------------
	ifstream modelFile;
	modelFile.open("sphere.txt");
	int sphere_numLines = 0;
	modelFile >> sphere_numLines;
	float *sphere = new float[sphere_numLines];
	for (int i = 0; i < sphere_numLines; i++) {
		modelFile >> sphere[i];
	}
	printf("sphere numLines: %d\n", sphere_numLines);
	int sphere_numVerts = sphere_numLines / 8;
	modelFile.close();

	//Allocate memory on the graphics card to store geometry (vertex buffer object)
	GLuint vbo_sphere;
	glGenBuffers(1, &vbo_sphere);  //Create 1 buffer called vbo
	glBindBuffer(GL_ARRAY_BUFFER, vbo_sphere); //Set the vbo as the active array buffer (Only one buffer can be active at a time)
	glBufferData(GL_ARRAY_BUFFER, sphere_numLines * sizeof(float), sphere, GL_STATIC_DRAW); //upload vertices to vbo

	GLuint vao_sphere;
	glGenVertexArrays(1, &vao_sphere); //Create a VAO
	glBindVertexArray(vao_sphere); //Bind the above created VAO to the current context

	//Load the vertex Shader
	GLuint SphereVertexShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(SphereVertexShader, 1, &vertexSource, NULL);
	glCompileShader(SphereVertexShader);

	//Let's double check the shader compiled 
	GLint status;
	glGetShaderiv(SphereVertexShader, GL_COMPILE_STATUS, &status);
	if (!status) {
		char buffer[512];
		glGetShaderInfoLog(SphereVertexShader, 512, NULL, buffer);
		SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR,
			"Compilation Error",
			"Failed to Compile: Check Consol Output.",
			NULL);
		printf("Vertex Shader Compile Failed. Info:\n\n%s\n", buffer);
	}

	GLuint SphereFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(SphereFragmentShader, 1, &fragmentSource, NULL);
	glCompileShader(SphereFragmentShader);

	//Double check the shader compiled 
	glGetShaderiv(SphereFragmentShader, GL_COMPILE_STATUS, &status);
	if (!status) {
		char buffer[512];
		glGetShaderInfoLog(SphereFragmentShader, 512, NULL, buffer);
		SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR,
			"Compilation Error",
			"Failed to Compile: Check Consol Output.",
			NULL);
		printf("Fragment Shader Compile Failed. Info:\n\n%s\n", buffer);
	}

	//Join the vertex and fragment shaders together into one program
	GLuint SphereShaderProgram = glCreateProgram();
	glAttachShader(SphereShaderProgram, SphereVertexShader);
	glAttachShader(SphereShaderProgram, SphereFragmentShader);
	glBindFragDataLocation(SphereShaderProgram, 0, "outColor"); // set output
	glLinkProgram(SphereShaderProgram); //run the linker
	glUseProgram(SphereShaderProgram); //Set the active shader (only one can be used at a time)

	//Tell OpenGL how to set fragment shader input 
	posAttrib = glGetAttribLocation(SphereShaderProgram, "position");
	glVertexAttribPointer(posAttrib, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), 0);
	//Attribute, vals/attrib., type, normalized?, stride, offset
	//Binds to VBO current GL_ARRAY_BUFFER 
	glEnableVertexAttribArray(posAttrib);

	nolAttrib = glGetAttribLocation(SphereShaderProgram, "inNormal");
	glVertexAttribPointer(nolAttrib, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(5 * sizeof(float)));
	glEnableVertexAttribArray(nolAttrib);

	glBindVertexArray(0); //Unbind the VAO

	//// load edges in the graph
	//vector<float>edge_;
	//for (int i = 0; i < roadmap.graph.node.size(); i++) {
	//	for (int j = i; j < roadmap.graph.node.size(); j++) {
	//		if (roadmap.graph.arc[i][j] != 0) {
	//			edge_.push_back(roadmap.graph.node[i].x);
	//			edge_.push_back(roadmap.graph.node[i].y);
	//			edge_.push_back(roadmap.graph.node[i].z);
	//			edge_.push_back(roadmap.graph.node[j].x);
	//			edge_.push_back(roadmap.graph.node[j].y);
	//			edge_.push_back(roadmap.graph.node[j].z);
	//		}
	//	}
	//}
	//float *edge = (float*)malloc(edge_.size() * sizeof(float));
	//for (int i = 0; i < edge_.size(); i++) {
	//	edge[i] = edge_[i];
	//}

	//printf("edge_size: %d\n", edge_.size());
	//printf("edge_num: %d\n", edge_.size()/3);

	//GLuint vbo_edge;
	//glGenBuffers(1, &vbo_edge);  //Create 1 buffer called vbo
	//glBindBuffer(GL_ARRAY_BUFFER, vbo_edge); //Set the vbo as the active array buffer (Only one buffer can be active at a time)
	//glBufferData(GL_ARRAY_BUFFER, edge_.size() * sizeof(float), edge, GL_STATIC_DRAW); //upload vertices to vbo

	//GLuint vao_edge;
	//glGenVertexArrays(1, &vao_edge); //Create a VAO
	//glBindVertexArray(vao_edge); //Bind the above created VAO to the current context

	////Load the vertex Shader
	//GLuint EdgeVertexShader = glCreateShader(GL_VERTEX_SHADER);
	//glShaderSource(EdgeVertexShader, 1, &vertexSource, NULL);
	//glCompileShader(EdgeVertexShader);

	////Let's double check the shader compiled 
	//glGetShaderiv(EdgeVertexShader, GL_COMPILE_STATUS, &status);
	//if (!status) {
	//	char buffer[512];
	//	glGetShaderInfoLog(EdgeVertexShader, 512, NULL, buffer);
	//	SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR,
	//		"Compilation Error",
	//		"Failed to Compile: Check Consol Output.",
	//		NULL);
	//	printf("Vertex Shader Compile Failed. Info:\n\n%s\n", buffer);
	//}

	//GLuint EdgeFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	//glShaderSource(EdgeFragmentShader, 1, &fragmentSource, NULL);
	//glCompileShader(EdgeFragmentShader);

	////Double check the shader compiled 
	//glGetShaderiv(EdgeFragmentShader, GL_COMPILE_STATUS, &status);
	//if (!status) {
	//	char buffer[512];
	//	glGetShaderInfoLog(EdgeFragmentShader, 512, NULL, buffer);
	//	SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR,
	//		"Compilation Error",
	//		"Failed to Compile: Check Consol Output.",
	//		NULL);
	//	printf("Fragment Shader Compile Failed. Info:\n\n%s\n", buffer);
	//}

	////Join the vertex and fragment shaders together into one program
	//GLuint EdgeShaderProgram = glCreateProgram();
	//glAttachShader(EdgeShaderProgram, EdgeVertexShader);
	//glAttachShader(EdgeShaderProgram, EdgeFragmentShader);
	//glBindFragDataLocation(EdgeShaderProgram, 0, "outColor"); // set output
	//glLinkProgram(EdgeShaderProgram); //run the linker
	//glUseProgram(EdgeShaderProgram); //Set the active shader (only one can be used at a time)

	////Tell OpenGL how to set fragment shader input 
	//posAttrib = glGetAttribLocation(EdgeShaderProgram, "position");
	//glVertexAttribPointer(posAttrib, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
	////Attribute, vals/attrib., type, normalized?, stride, offset
	////Binds to VBO current GL_ARRAY_BUFFER 
	//glEnableVertexAttribArray(posAttrib);

	///*glm::vec3 inColor = glm::vec3(0.0f, 0.0f, 0.0f);
	//uniProj = glGetUniformLocation(SphereShaderProgram, "inColor");
	//glUniform3f(uniColor, inColor.r, inColor.g, inColor.b);*/

	//glBindVertexArray(0); //Unbind the VAO

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glLineWidth(3.0f);

	//Event Loop (Loop forever processing each event as fast as possible)
	SDL_Event windowEvent;
	bool quit = false;
	srand(time(NULL));

	float lastTime;// = SDL_GetTicks() / 1000.f;
	float dt = 0;

	//set parameters for camera
	float movestep = 0.1;
	float anglestep = 0.1;
	glm::vec3 camera_position = glm::vec3(0.0f, 0.0f, 25.0f);  //Cam Position
	glm::vec3 look_point = glm::vec3(0.0f, 0.0f, 0.0f);  //Look at point
	glm::vec3 up_vector = glm::vec3(0.0f, 1.0f, 0.0f); //Up
	glm::vec3 move_vector = glm::vec3(0.0f, 0.0f, 0.0f);

	glm::mat4 proj = glm::perspective(3.14f / 4, aspect, 0.1f, 10000.0f); //FOV, aspect, near, far

	glm::vec3 inColor;

	float timecost = 0.0f;
	int frame = 0;
	bool notpause = false;
	//passed.push_back(0);

	while (!quit) {
		while (SDL_PollEvent(&windowEvent)) {
			if (windowEvent.type == SDL_QUIT) quit = true; //Exit event loop
			//List of keycodes: https://wiki.libsdl.org/SDL_Keycode - You can catch many special keys
			//Scancode referes to a keyboard position, keycode referes to the letter (e.g., EU keyboards)
			if (windowEvent.type == SDL_KEYUP && windowEvent.key.keysym.sym == SDLK_ESCAPE)
				quit = true; ; //Exit event loop
			if (windowEvent.type == SDL_KEYUP && windowEvent.key.keysym.sym == SDLK_f) //If "f" is pressed
				fullscreen = !fullscreen;
			SDL_SetWindowFullscreen(window, fullscreen ? SDL_WINDOW_FULLSCREEN : 0); //Set to full screen

			//user-controlled camera
			if ((windowEvent.type == SDL_KEYDOWN && windowEvent.key.keysym.sym == SDLK_UP) || \
				(windowEvent.type == SDL_KEYDOWN && windowEvent.key.keysym.sym == SDLK_w)) {
				//move up
				move_vector = glm::normalize(look_point - camera_position)*movestep;
				camera_position = camera_position + move_vector;
				look_point = look_point + move_vector;
			}
			if ((windowEvent.type == SDL_KEYDOWN && windowEvent.key.keysym.sym == SDLK_DOWN) || \
				(windowEvent.type == SDL_KEYDOWN && windowEvent.key.keysym.sym == SDLK_s)) {
				//move down
				move_vector = glm::normalize(look_point - camera_position)*movestep;
				camera_position = camera_position - move_vector;
				look_point = look_point - move_vector;
			}
			if ((windowEvent.type == SDL_KEYDOWN && windowEvent.key.keysym.sym == SDLK_LEFT) || \
				(windowEvent.type == SDL_KEYDOWN && windowEvent.key.keysym.sym == SDLK_a)) {
				//move left
				move_vector = glm::normalize(look_point - camera_position);
				move_vector = glm::cross(up_vector, move_vector)*movestep;
				camera_position = camera_position + move_vector;
				look_point = look_point + move_vector;
			}
			if ((windowEvent.type == SDL_KEYDOWN && windowEvent.key.keysym.sym == SDLK_RIGHT) || \
				(windowEvent.type == SDL_KEYDOWN && windowEvent.key.keysym.sym == SDLK_d)) {
				//move right
				move_vector = glm::normalize(look_point - camera_position);
				move_vector = glm::cross(up_vector, move_vector)*movestep;
				camera_position = camera_position - move_vector;
				look_point = look_point - move_vector;
			}
			if (windowEvent.type == SDL_MOUSEWHEEL && windowEvent.wheel.y == 1) {// scroll up
				//turn left
				move_vector = glm::rotateZ(look_point - camera_position, anglestep);
				look_point = camera_position + move_vector;
			}
			if (windowEvent.type == SDL_MOUSEWHEEL && windowEvent.wheel.y == -1) {// scroll up
				//turn right
				move_vector = glm::rotateZ(look_point - camera_position, -anglestep);
				look_point = camera_position + move_vector;
			}

			//add obstacle
			if (windowEvent.type == SDL_KEYDOWN && windowEvent.key.keysym.sym == SDLK_F1) {
				Obstacle obs;
				obs.SetPosition(glm::vec3(randf(), randf(), 0.0f));
				obs.SetRadius(2.5);
				roadmap.obstacles.push_back(obs);
				roadmap.obstacle_num++;
				
				for (int i = 0; i < roadmap.agent.size(); i++) {
					roadmap.agent[i].start = roadmap.agent[i].position;
					roadmap.agent[i].current_pos = 1;
					roadmap.graph.InsertNode(roadmap.agent[i].start);
				}

				bool allpathgot = false;
				while (!allpathgot) {
					roadmap.GeneratePoint(n);
					allpathgot = roadmap.FindAllPath(heuristic);
				}
			}
			//move obstacle
			if (windowEvent.type == SDL_KEYDOWN && windowEvent.key.keysym.sym == SDLK_F2) {
				for (int i = 0; i < roadmap.obstacles.size(); i++) {
					roadmap.obstacles[i].SetPosition(glm::vec3(7*randf(), 7*randf(), 0.0f));
				}

				for (int i = 0; i < roadmap.agent.size(); i++) {
					roadmap.agent[i].start = roadmap.agent[i].position;
					roadmap.agent[i].current_pos = 1;
					roadmap.agent[i].velocity = glm::vec3(0.0f);
					roadmap.graph.InsertNode(roadmap.agent[i].start);
				}

				bool allpathgot = false;
				while (!allpathgot) {
					allpathgot = roadmap.FindAllPath(heuristic);
					roadmap.GeneratePoint(n);
				}
			}


			if (windowEvent.type == SDL_KEYDOWN && windowEvent.key.keysym.sym == SDLK_p) {
				notpause = true;
				lastTime = SDL_GetTicks() / 1000.f;
			}

		}

		// Clear the screen to default color
		glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		if (notpause) {
			if (!saveOutput) dt = (SDL_GetTicks() / 1000.f) - lastTime;
			lastTime = SDL_GetTicks() / 1000.f;
			if (saveOutput) dt += .07; //Fix framerate at 14 FPS
									   //printf("frame_id:%d\ttimecost:%f\n", frame, timecost);
		}
		

		glm::mat4 view = glm::lookAt(camera_position, look_point, up_vector);

		glUseProgram(SphereShaderProgram);
		//glBindVertexArray(vao_sphere);
		uniProj = glGetUniformLocation(SphereShaderProgram, "proj");
		glUniformMatrix4fv(uniProj, 1, GL_FALSE, glm::value_ptr(proj));
		
		uniView = glGetUniformLocation(SphereShaderProgram, "view");
		glUniformMatrix4fv(uniView, 1, GL_FALSE, glm::value_ptr(view));		

		// draw the obstacle	
		for (int i = 0; i < roadmap.obstacle_num; i++) {
			inColor = glm::vec3(1.0f, 0.0f, 0.0f);
			uniProj = glGetUniformLocation(SphereShaderProgram, "inColor");
			glUniform3f(uniColor, inColor.r, inColor.g, inColor.b);

			glm::mat4 sphere_model(1.0f);
			sphere_model = glm::translate(sphere_model, roadmap.obstacles[i].GetPosition());
			sphere_model = glm::scale(sphere_model, glm::vec3(roadmap.obstacles[i].GetRadius()*2.0));
			uniModel = glGetUniformLocation(SphereShaderProgram, "model");
			glUniformMatrix4fv(uniModel, 1, GL_FALSE, glm::value_ptr(sphere_model));

			glBindVertexArray(vao_sphere);
			glDrawArrays(GL_TRIANGLES, 0, sphere_numVerts); //(Primitives, Which VBO, Number of vertices)
		}
		
		// draw path for agents
		for (int i = 0; i < roadmap.agent.size(); i++) {
			glUseProgram(SphereShaderProgram);
			inColor = glm::vec3(0.0f, 1.0f, 1.0f);
			uniProj = glGetUniformLocation(SphereShaderProgram, "inColor");
			glUniform3f(uniColor, inColor.r, inColor.g, inColor.b);

			glm::mat4 agent_model(1.0f);
			agent_model = glm::translate(agent_model, roadmap.agent[i].position);
			agent_model = glm::scale(agent_model, glm::vec3(roadmap.agent[i].size));
			uniModel = glGetUniformLocation(SphereShaderProgram, "model");
			glUniformMatrix4fv(uniModel, 1, GL_FALSE, glm::value_ptr(agent_model));

			glBindVertexArray(vao_sphere);
			glDrawArrays(GL_TRIANGLES, 0, sphere_numVerts); //(Primitives, Which VBO, Number of vertices)
		}

		//roadmap.UpdateAgentPosition_TTC(dt, roadmap.graph.node);
		//roadmap.UpdateAgentPosition_Boid(dt, roadmap.graph.node);
		//roadmap.UpdateAgentPosition(dt);
		roadmap.UpdateAgentPosition_Smooth(dt);

		//draw graph
		////draw vertices		
		for (int i = 0; i < roadmap.graph.node.size(); i++) {
			inColor = glm::vec3(0.0f, 0.0f, 0.0f);
			uniProj = glGetUniformLocation(SphereShaderProgram, "inColor");
			glUniform3f(uniColor, inColor.r, inColor.g, inColor.b);

			glm::mat4 sphere_model(1.0f);
			sphere_model = glm::translate(sphere_model, roadmap.graph.node[i]);
			sphere_model = glm::scale(sphere_model, glm::vec3(0.5));
			uniModel = glGetUniformLocation(SphereShaderProgram, "model");
			glUniformMatrix4fv(uniModel, 1, GL_FALSE, glm::value_ptr(sphere_model));
			glBindVertexArray(vao_sphere);
			glDrawArrays(GL_TRIANGLES, 0, sphere_numVerts); //(Primitives, Which VBO, Number of vertices)
			glBindVertexArray(0);
		}

		//draw edges
		// load edges in the graph
		vector<float>edge_;
		for (int i = 0; i < roadmap.graph.node.size(); i++) {
			for (int j = i; j < roadmap.graph.node.size(); j++) {
				if (roadmap.graph.arc[i][j] != 0) {
					edge_.push_back(roadmap.graph.node[i].x);
					edge_.push_back(roadmap.graph.node[i].y);
					edge_.push_back(roadmap.graph.node[i].z);
					edge_.push_back(roadmap.graph.node[j].x);
					edge_.push_back(roadmap.graph.node[j].y);
					edge_.push_back(roadmap.graph.node[j].z);
				}
			}
		}
		float *edge = (float*)malloc(edge_.size() * sizeof(float));
		for (int i = 0; i < edge_.size(); i++) {
			edge[i] = edge_[i];
		}

		//printf("edge_size: %d\n", edge_.size());
		//printf("edge_num: %d\n", edge_.size() / 3);

		GLuint vbo_edge;
		glGenBuffers(1, &vbo_edge);  //Create 1 buffer called vbo
		glBindBuffer(GL_ARRAY_BUFFER, vbo_edge); //Set the vbo as the active array buffer (Only one buffer can be active at a time)
		glBufferData(GL_ARRAY_BUFFER, edge_.size() * sizeof(float), edge, GL_STATIC_DRAW); //upload vertices to vbo

		GLuint vao_edge;
		glGenVertexArrays(1, &vao_edge); //Create a VAO
		glBindVertexArray(vao_edge); //Bind the above created VAO to the current context

									 //Load the vertex Shader
		GLuint EdgeVertexShader = glCreateShader(GL_VERTEX_SHADER);
		glShaderSource(EdgeVertexShader, 1, &vertexSource, NULL);
		glCompileShader(EdgeVertexShader);

		//Let's double check the shader compiled 
		glGetShaderiv(EdgeVertexShader, GL_COMPILE_STATUS, &status);
		if (!status) {
			char buffer[512];
			glGetShaderInfoLog(EdgeVertexShader, 512, NULL, buffer);
			SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR,
				"Compilation Error",
				"Failed to Compile: Check Consol Output.",
				NULL);
			printf("Vertex Shader Compile Failed. Info:\n\n%s\n", buffer);
		}

		GLuint EdgeFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
		glShaderSource(EdgeFragmentShader, 1, &fragmentSource, NULL);
		glCompileShader(EdgeFragmentShader);

		//Double check the shader compiled 
		glGetShaderiv(EdgeFragmentShader, GL_COMPILE_STATUS, &status);
		if (!status) {
			char buffer[512];
			glGetShaderInfoLog(EdgeFragmentShader, 512, NULL, buffer);
			SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR,
				"Compilation Error",
				"Failed to Compile: Check Consol Output.",
				NULL);
			printf("Fragment Shader Compile Failed. Info:\n\n%s\n", buffer);
		}

		//Join the vertex and fragment shaders together into one program
		GLuint EdgeShaderProgram = glCreateProgram();
		glAttachShader(EdgeShaderProgram, EdgeVertexShader);
		glAttachShader(EdgeShaderProgram, EdgeFragmentShader);
		glBindFragDataLocation(EdgeShaderProgram, 0, "outColor"); // set output
		glLinkProgram(EdgeShaderProgram); //run the linker
		glUseProgram(EdgeShaderProgram); //Set the active shader (only one can be used at a time)

										 //Tell OpenGL how to set fragment shader input 
		posAttrib = glGetAttribLocation(EdgeShaderProgram, "position");
		glVertexAttribPointer(posAttrib, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
		//Attribute, vals/attrib., type, normalized?, stride, offset
		//Binds to VBO current GL_ARRAY_BUFFER 
		glEnableVertexAttribArray(posAttrib);

		/*glm::vec3 inColor = glm::vec3(0.0f, 0.0f, 0.0f);
		uniProj = glGetUniformLocation(SphereShaderProgram, "inColor");
		glUniform3f(uniColor, inColor.r, inColor.g, inColor.b);*/

		glBindVertexArray(0); //Unbind the VAO

		glUseProgram(EdgeShaderProgram);
		uniProj = glGetUniformLocation(EdgeShaderProgram, "proj");
		glUniformMatrix4fv(uniProj, 1, GL_FALSE, glm::value_ptr(proj));

		uniView = glGetUniformLocation(EdgeShaderProgram, "view");
		glUniformMatrix4fv(uniView, 1, GL_FALSE, glm::value_ptr(view));

		glm::mat4 edge_model(1.0f);
		uniModel = glGetUniformLocation(EdgeShaderProgram, "model");
		glUniformMatrix4fv(uniModel, 1, GL_FALSE, glm::value_ptr(edge_model));
		glBindVertexArray(vao_edge);
		glDrawArrays(GL_LINES, 0, edge_.size()/3); //(Primitives, Which VBO, Number of vertices)
		glBindVertexArray(0);

		//draw ground
		glUseProgram(ClothShaderProgram);
		uniProj = glGetUniformLocation(ClothShaderProgram, "proj");
		glUniformMatrix4fv(uniProj, 1, GL_FALSE, glm::value_ptr(proj));

		uniView = glGetUniformLocation(ClothShaderProgram, "view");
		glUniformMatrix4fv(uniView, 1, GL_FALSE, glm::value_ptr(view));

		glm::mat4 cloth_model(1.0f);
		uniModel = glGetUniformLocation(ClothShaderProgram, "model");
		cloth_model = glm::scale(cloth_model, glm::vec3(10));
		glUniformMatrix4fv(uniModel, 1, GL_FALSE, glm::value_ptr(cloth_model));

		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, tex0);
		glUniform1i(glGetUniformLocation(ClothShaderProgram, "tex0"), 0);

		glBindVertexArray(vao_cloth);
		GLint uniTexID = glGetUniformLocation(ClothShaderProgram, "texID");
		glUniform1i(uniTexID, 0); //Set texture ID to use 
		glDrawArrays(GL_TRIANGLES, 0, cloth_numVerts); //(Primitives, Which VBO, Number of vertices)
		glBindVertexArray(0);

		//compute fps
		frame++;
		timecost += dt;

		//if (saveOutput) Win2PPM(screen_width, screen_height);

		SDL_GL_SwapWindow(window); //Double buffering
	}

	//Clean Up
	glDeleteProgram(ClothShaderProgram);
	glDeleteProgram(SphereShaderProgram);
	glDeleteShader(SphereFragmentShader);
	glDeleteShader(SphereVertexShader);
	glDeleteBuffers(1, &vbo_cloth);
	glDeleteVertexArrays(1, &vao_cloth);
	glDeleteBuffers(1, &vbo_sphere);
	glDeleteVertexArrays(1, &vao_sphere);

	SDL_GL_DeleteContext(context);
	SDL_Quit();

	free(sphere);
	//free(edge);
	return 0;
}

// Create a NULL-terminated string by reading the provided file
static char* readShaderSource(const char* shaderFile) {
	FILE *fp;
	long length;
	char *buffer;

	// open the file containing the text of the shader code
	fp = fopen(shaderFile, "r");

	// check for errors in opening the file
	if (fp == NULL) {
		printf("can't open shader source file %s\n", shaderFile);
		return NULL;
	}

	// determine the file size
	fseek(fp, 0, SEEK_END); // move position indicator to the end of the file;
	length = ftell(fp);  // return the value of the current position

						 // allocate a buffer with the indicated number of bytes, plus one
	buffer = new char[length + 1];

	// read the appropriate number of bytes from the file
	fseek(fp, 0, SEEK_SET);  // move position indicator to the start of the file
	fread(buffer, 1, length, fp); // read all of the bytes

	// append a NULL character to indicate the end of the string
	buffer[length] = '\0';

	// close the file
	fclose(fp);

	// return the string
	return buffer;
}

// Create a GLSL program object from vertex and fragment shader files
GLuint InitShader(const char* vShaderFileName, const char* fShaderFileName) {
	GLuint vertex_shader, fragment_shader;
	GLchar *vs_text, *fs_text;
	GLuint program;

	// check GLSL version
	printf("GLSL version: %s\n\n", glGetString(GL_SHADING_LANGUAGE_VERSION));

	// Create shader handlers
	vertex_shader = glCreateShader(GL_VERTEX_SHADER);
	fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);

	// Read source code from shader files
	vs_text = readShaderSource(vShaderFileName);
	fs_text = readShaderSource(fShaderFileName);

	// error check
	if (vs_text == NULL) {
		printf("Failed to read from vertex shader file %s\n", vShaderFileName);
		exit(1);
	}
	else if (DEBUG_ON) {
		printf("Vertex Shader:\n=====================\n");
		printf("%s\n", vs_text);
		printf("=====================\n\n");
	}
	if (fs_text == NULL) {
		printf("Failed to read from fragent shader file %s\n", fShaderFileName);
		exit(1);
	}
	else if (DEBUG_ON) {
		printf("\nFragment Shader:\n=====================\n");
		printf("%s\n", fs_text);
		printf("=====================\n\n");
	}

	// Load Vertex Shader
	const char *vv = vs_text;
	glShaderSource(vertex_shader, 1, &vv, NULL);  //Read source
	glCompileShader(vertex_shader); // Compile shaders

									// Check for errors
	GLint  compiled;
	glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &compiled);
	if (!compiled) {
		printf("Vertex shader failed to compile:\n");
		if (DEBUG_ON) {
			GLint logMaxSize, logLength;
			glGetShaderiv(vertex_shader, GL_INFO_LOG_LENGTH, &logMaxSize);
			printf("printing error message of %d bytes\n", logMaxSize);
			char* logMsg = new char[logMaxSize];
			glGetShaderInfoLog(vertex_shader, logMaxSize, &logLength, logMsg);
			printf("%d bytes retrieved\n", logLength);
			printf("error message: %s\n", logMsg);
			delete[] logMsg;
		}
		exit(1);
	}

	// Load Fragment Shader
	const char *ff = fs_text;
	glShaderSource(fragment_shader, 1, &ff, NULL);
	glCompileShader(fragment_shader);
	glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &compiled);

	//Check for Errors
	if (!compiled) {
		printf("Fragment shader failed to compile\n");
		if (DEBUG_ON) {
			GLint logMaxSize, logLength;
			glGetShaderiv(fragment_shader, GL_INFO_LOG_LENGTH, &logMaxSize);
			printf("printing error message of %d bytes\n", logMaxSize);
			char* logMsg = new char[logMaxSize];
			glGetShaderInfoLog(fragment_shader, logMaxSize, &logLength, logMsg);
			printf("%d bytes retrieved\n", logLength);
			printf("error message: %s\n", logMsg);
			delete[] logMsg;
		}
		exit(1);
	}

	// Create the program
	program = glCreateProgram();

	// Attach shaders to program
	glAttachShader(program, vertex_shader);
	glAttachShader(program, fragment_shader);

	// Link and set program to use
	glLinkProgram(program);

	return program;
}