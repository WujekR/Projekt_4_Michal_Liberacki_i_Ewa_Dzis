#pragma once
#include <iostream>
#include <memory>
#include <matplot/matplot.h>

#include <Eigen/Dense>
#include <SDL.h>
#include <SDL2_gfx/SDL2_gfxPrimitives.h>

#include "planar_quadrotor.h"
#include "planar_quadrotor_visualizer.h"
#include "lqr.h"
namespace mp = matplot;
int init(std::shared_ptr<SDL_Window>& gWindow, std::shared_ptr<SDL_Renderer>& gRenderer, const int SCREEN_WIDTH, const int SCREEN_HEIGHT);
