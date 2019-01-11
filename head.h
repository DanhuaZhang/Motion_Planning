#pragma once

#include "glad/glad.h"  //Include order can matter here
#ifndef _WIN32
#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>
#else
#include <SDL.h>
#include <SDL_opengl.h>
#endif

#include <cstdio>
#include <cstdlib>

#define INFINITY 10000000000
#define MAX_CHARACTER 1024
#define GLM_FORCE_RADIANS

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/rotate_vector.hpp"
#include <time.h>
#include <vector>
#include <queue>
#include <string>
#include <fstream>
#include <algorithm>

using namespace std;
