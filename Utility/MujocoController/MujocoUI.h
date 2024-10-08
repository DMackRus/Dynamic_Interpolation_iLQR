//
// Created by davem on 16/02/2022.
//

#ifndef MUJOCO_SANDBOX_MUJOCOUI_H
#define MUJOCO_SANDBOX_MUJOCOUI_H

#include "MujocoController.h"

void scroll(GLFWwindow* window, double xoffset, double yoffset);
void mouse_move(GLFWwindow* window, double xpos, double ypos);
void mouse_button(GLFWwindow* window, int button, int act, int mods);
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);

void setupMujocoWorld(double timestep, const char *fileName);
void render();
void render_simpleTest();
void renderMPCAfter();
void renderOnce(mjData *renderData);
void initMujoco(double timestep, const char *fileName);
void updateScreen();

#endif //MUJOCO_SANDBOX_MUJOCOUI_H
