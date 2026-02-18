// ----------------------------------------------------------------------------
// main.cpp
//
//  Created on: 13 Dec 2020
//      Author: Kiwon Um
//        Mail: kiwon.um@telecom-paris.fr
//
// Description: Lab - Rigid Body Simulator (DO NOT distribute!)
//
// Copyright 2020-2026 Kiwon Um
//
// The copyright to the computer program(s) herein is the property of Kiwon Um,
// Telecom Paris, France. The program(s) may be used and/or copied only with
// the written permission of Kiwon Um or in accordance with the terms and
// conditions stipulated in the agreement/contract under which the program(s)
// have been supplied.
// ----------------------------------------------------------------------------

#define _USE_MATH_DEFINES

#include <glad/gl.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <ios>
#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include <algorithm>
#include <exception>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// #include "Error.h" // OpenGL 4.3 or later
#include "ShaderProgram.h"
#include "Camera.h"
#include "Mesh.h"

#include "RigidSolver.hpp"

// window parameters
GLFWwindow *g_window = nullptr;
int g_windowWidth = 1024;
int g_windowHeight = 768;

// pointer to the current camera model
std::shared_ptr<Camera> g_cam;

// camera control variables
float g_meshScale = 1.0; // to update based on the mesh size, so that navigation runs at scale
bool g_rotatingP = false;
bool g_panningP = false;
bool g_zoomingP = false;
double g_baseX = 0.0, g_baseY = 0.0;
glm::vec3 g_baseTrans(0.0);
glm::vec3 g_baseRot(0.0);

// Variables pour orbite caméra
float g_camOrbitRadius = 6.0f;
float g_camOrbitYaw = 0.0f;
float g_camOrbitPitch = 0.0f;

// camera follow mode
bool g_cameraFollowsBall = false;

// timer
float g_appTimer = 0.0;
float g_appTimerLastClockTime;
bool g_appTimerStoppedP = true;

// textures
unsigned int g_availableTextureSlot = 0;

GLuint g_albedoTex;
unsigned int g_albedoTexOnGPU;

GLuint g_normalTex;
unsigned int g_normalTexOnGPU;

// Couleur du soleil (utilisée pour la lumière et la sphère)
const glm::vec3 sunColor = glm::vec3(0.95, 0.95, 0.85); // presque blanc, légèrement chaud

// Rayon global de la sphère (balle)
const float g_ballRadius = 0.03f; // rayon par défaut, ajustez si besoin
// Position initiale de la balle (utilisée pour la caméra libre)
glm::vec3 g_ballInitPos = glm::vec3(posInit.x, posInit.y, posInit.z);

GLuint loadTextureFromFileToGPU(const std::string &filename)
{
  int width, height, numComponents;
  // Loading the image in CPU memory using stbd_image
  unsigned char *data = stbi_load(
    filename.c_str(),
    &width,
    &height,
    &numComponents, // 1 for a 8 bit greyscale image, 3 for 24bits RGB image, 4 for 32bits RGBA image
    0);

  // Create a texture in GPU memory
  GLuint texID;
  glGenTextures(1, &texID);
  glBindTexture(GL_TEXTURE_2D, texID);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  // Uploading the image data to GPU memory
  glTexImage2D(
    GL_TEXTURE_2D,
    0,
    (numComponents == 1 ? GL_RED : numComponents == 3 ? GL_RGB : GL_RGBA), // For greyscale images, we store them in the RED channel
    width,
    height,
    0,
    (numComponents == 1 ? GL_RED : numComponents == 3 ? GL_RGB : GL_RGBA), // For greyscale images, we store them in the RED channel
    GL_UNSIGNED_BYTE,
    data);

  // Generating mipmaps for filtered texture fetch
  glGenerateMipmap(GL_TEXTURE_2D);

  // Freeing the now useless CPU memory
  stbi_image_free(data);
  glBindTexture(GL_TEXTURE_2D, 0); // unbind the texture
  return texID;
}

struct Light {
  glm::vec3 position;
  glm::vec3 color;
  float intensity;
};


struct Scene {
  Light light;

  RigidSolver solver = RigidSolver(nullptr, Vec3f(0, -9.81, 0));
  std::shared_ptr<BodyAttributes> rigidAtt = nullptr;

  // meshes
  std::shared_ptr<Mesh> rigid = nullptr;
  std::shared_ptr<Mesh> plane = nullptr;
  std::shared_ptr<Mesh> table = nullptr; // table de pingpong
  std::shared_ptr<Mesh> net = nullptr; // filet

  // transformation matrices
  glm::mat4 rigidMat = glm::mat4(1.0);
  glm::mat4 planeMat = glm::mat4(1.0);
  glm::mat4 floorMat = glm::mat4(1.0);
  glm::mat4 tableMat = glm::mat4(1.0); // matrice de la table
  glm::mat4 netMat = glm::mat4(1.0); // matrice du filet

  float netHeight = 0.1f; // Hauteur du filet

  glm::vec3 scene_center = glm::vec3(0);
  float scene_radius = 1.f;

  // shaders to render the meshes
  std::shared_ptr<ShaderProgram> mainShader;

  // useful for debug
  bool saveScreenShot = false;
  int savedCnt = 0;

  void resetSim()
  {
    *rigidAtt = Sphere(0.6f); // sphère de rayon 0.6
    rigidAtt->X = posInit;
    rigidAtt->V = Vec3f(0,0,0);
    rigidAtt->P = Vec3f(0,0,0);
    rigidAtt->L = Vec3f(0,0,0);
    rigidAtt->F = Vec3f(0,0,0);
    rigidAtt->q = Quaternion::identity();
    rigidAtt->R = rigidAtt->q.toMatrix();
    rigidAtt->Iinv = rigidAtt->R * rigidAtt->I0inv * rigidAtt->R.transposed();
    solver.init(rigidAtt.get());
    solver.gameOver = false;
    solver.lastWallTouched = 0;
    rigidMat = rigidAtt->worldMat();
  }

  void render()
  {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, g_windowWidth, g_windowHeight);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // clear the color and z buffers.

    //glDisable(GL_CULL_FACE);    // or
    glCullFace(GL_BACK);

    mainShader->use();

    // camera
    mainShader->set("camPos", g_cam->getPosition());
    mainShader->set("viewMat", g_cam->computeViewMatrix());
    mainShader->set("projMat", g_cam->computeProjectionMatrix());
    mainShader->set("fov", g_cam->getFov());
    mainShader->set("aspect", g_cam->getAspectRatio());

    // light
    mainShader->set(std::string("lightSrc.position"), light.position);
    mainShader->set(std::string("lightSrc.color"), light.color);
    mainShader->set(std::string("lightSrc.intensity"), light.intensity);

    // back-wall
    mainShader->set("material.albedo", glm::vec3(0.36, 0.25, 0.20)); // marron
    mainShader->set("material.albedoTexLoaded", 0);
    mainShader->set("material.normalTex", (int)g_normalTexOnGPU);
    mainShader->set("material.normalTexLoaded", 1);
    mainShader->set("modelMat", planeMat);
    mainShader->set("normMat", glm::mat3(glm::inverseTranspose(planeMat)));
    plane->render();

    // floor
    mainShader->set("material.albedo", glm::vec3(0.8, 0.8, 0.9));
    mainShader->set("material.albedoTexLoaded", 0);
    mainShader->set("material.normalTexLoaded", 0);
    mainShader->set("modelMat", floorMat);
    mainShader->set("normMat", glm::mat3(glm::inverseTranspose(floorMat)));
    plane->render();

    // rigid (SOLEIL)
    mainShader->set("material.albedo", sunColor); // couleur unifiée
    mainShader->set("material.albedoTexLoaded", 0); // ignore la texture
    mainShader->set("material.normalTexLoaded", 0);
    mainShader->set("modelMat", rigidMat);
    mainShader->set("normMat", glm::mat3(glm::inverseTranspose(rigidMat)));
    rigid->render();

    // table de pingpong
    mainShader->set("material.albedo", glm::vec3(0.0, 0.35, 0.55)); // bleu typique table ping-pong
    mainShader->set("material.albedoTexLoaded", 0);
    mainShader->set("material.normalTexLoaded", 0);
    mainShader->set("modelMat", tableMat);
    mainShader->set("normMat", glm::mat3(glm::inverseTranspose(tableMat)));
    table->render();

    // filet de pingpong
    mainShader->set("material.albedo", glm::vec3(0.9, 0.9, 0.9)); // filet blanc
    mainShader->set("material.albedoTexLoaded", 0);
    mainShader->set("material.normalTexLoaded", 0);
    mainShader->set("modelMat", netMat);
    mainShader->set("normMat", glm::mat3(glm::inverseTranspose(netMat)));
    net->render();

    mainShader->stop(); 

    if(saveScreenShot) {
      std::stringstream fpath;
      fpath << "s" << std::setw(4) << std::setfill('0') << savedCnt++ << ".tga";

      std::cout << "Saving file " << fpath.str() << " ... " << std::flush;
      const short int w = g_windowWidth;
      const short int h = g_windowHeight;
      std::vector<int> buf(w*h*3, 0);
      glReadPixels(0, 0, w, h, GL_BGR, GL_UNSIGNED_BYTE, &(buf[0]));

      FILE *out = fopen(fpath.str().c_str(), "wb");
      short TGAhead[] = {0, 2, 0, 0, 0, 0, w, h, 24};
      fwrite(&TGAhead, sizeof(TGAhead), 1, out);
      fwrite(&(buf[0]), 3*w*h, 1, out);
      fclose(out);
      saveScreenShot = false;

      std::cout << "Done" << std::endl;
    }
  }
};

Scene g_scene;


// Ajout pour le quad plein écran
GLuint quadVAO = 0, quadVBO = 0;

void initFullscreenQuad() {
    float quadVertices[] = {
        // positions
        -1.0f, -1.0f,
         1.0f, -1.0f,
         1.0f,  1.0f,
        -1.0f, -1.0f,
         1.0f,  1.0f,
        -1.0f,  1.0f
    };
    glGenVertexArrays(1, &quadVAO);
    glGenBuffers(1, &quadVBO);
    glBindVertexArray(quadVAO);
    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), quadVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}


void printHelp()
{
  std::cout <<
    "> Help:" << std::endl <<
    "    Mouse commands:" << std::endl <<
    "    * Left button: rotate camera" << std::endl <<
    "    * Middle button: zoom" << std::endl <<
    "    * Right button: pan camera" << std::endl <<
    "    Keyboard commands:" << std::endl <<
    "    * H: print this help" << std::endl <<
    "    * P: toggle simulation" << std::endl <<
    "    * R: reset simulation" << std::endl <<
    "    * S: save a screenshot" << std::endl <<
    "    * W: wireframe rendering" << std::endl <<
    "    * F: surface rendering" << std::endl <<
    "    * ESC: quit the program" << std::endl;
}

// Executed each time the window is resized. Adjust the aspect ratio and the rendering viewport to the current window.
void windowSizeCallback(GLFWwindow *window, int width, int height)
{
  g_windowWidth = width;
  g_windowHeight = height;
  if (!g_cam) {
    std::cerr << "> [Warning] g_cam non initialisé lors du windowSizeCallback, callback ignoré." << std::endl;
    return;
  }
  g_cam->setAspectRatio(static_cast<float>(width)/static_cast<float>(height));
  // glViewport(0, 0, (GLint)width, (GLint)height); // TEST: commenté pour debug segfault
}

void keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
  if(action == GLFW_PRESS && key == GLFW_KEY_H) {
    printHelp();
  } else if(action == GLFW_PRESS && key == GLFW_KEY_R) {
    g_scene.resetSim();
  } else if(action == GLFW_PRESS && key == GLFW_KEY_S) {
    g_scene.saveScreenShot = true;
  } else if(action == GLFW_PRESS && key == GLFW_KEY_P) {
    g_appTimerStoppedP = !g_appTimerStoppedP;
    if(!g_appTimerStoppedP)
      g_appTimerLastClockTime = static_cast<float>(glfwGetTime());
  } else if(action == GLFW_PRESS && key == GLFW_KEY_W) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  } else if(action == GLFW_PRESS && key == GLFW_KEY_F) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  } else if(action == GLFW_PRESS && key == GLFW_KEY_ESCAPE) {
    glfwSetWindowShouldClose(window, true); // Closes the application if the escape key is pressed
  }
}

// Called each time the mouse cursor moves
void cursorPosCallback(GLFWwindow *window, double xpos, double ypos)
{
  int width, height;
  glfwGetWindowSize(window, &width, &height);
  const float normalizer = static_cast<float>((width + height)/2);
  const float dx = static_cast<float>((g_baseX - xpos) / normalizer);
  const float dy = static_cast<float>((ypos - g_baseY) / normalizer);
  if(g_rotatingP) {
    const glm::vec3 dRot(-dy*M_PI, dx*M_PI, 0.0);
    g_cam->setRotation(g_baseRot + dRot);
  } else if(g_panningP) {
    g_cam->setPosition(g_baseTrans + g_meshScale*glm::vec3(dx, dy, 0.0));
  } else if(g_zoomingP) {
    g_cam->setPosition(g_baseTrans + g_meshScale*glm::vec3(0.0, 0.0, dy));
  }
}

void mouseButtonCallback(GLFWwindow *window, int button, int action, int mods)
{
  if(button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
    if(!g_rotatingP) {
      g_rotatingP = true;
      glfwGetCursorPos(window, &g_baseX, &g_baseY);
      g_baseRot = g_cam->getRotation();
    }
  } else if(button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
    g_rotatingP = false;
  } else if(button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
    if(!g_panningP) {
      g_panningP = true;
      glfwGetCursorPos(window, &g_baseX, &g_baseY);
      g_baseTrans = g_cam->getPosition();
    }
  } else if(button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_RELEASE) {
    g_panningP = false;
  } else if(button == GLFW_MOUSE_BUTTON_MIDDLE && action == GLFW_PRESS) {
    if(!g_zoomingP) {
      g_zoomingP = true;
      glfwGetCursorPos(window, &g_baseX, &g_baseY);
      g_baseTrans = g_cam->getPosition();
    }
  } else if(button == GLFW_MOUSE_BUTTON_MIDDLE && action == GLFW_RELEASE) {
    g_zoomingP = false;
  }
}


void initGLFW()
{
  // Initialize GLFW, the library responsible for window management
  if(!glfwInit()) {
    std::cerr << "ERROR: Failed to init GLFW" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  // Before creating the window, set some option flags
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);

  // Create the window
  g_window = glfwCreateWindow(g_windowWidth, g_windowHeight, "A Simple Rigid Body Simulator", nullptr, nullptr);
  if(!g_window) {
    std::cerr << "ERROR: Failed to open window" << std::endl;
    glfwTerminate();
    std::exit(EXIT_FAILURE);
  }

  // Load the OpenGL context in the GLFW window using GLAD OpenGL wrangler
  glfwMakeContextCurrent(g_window);

  // not mandatory for all, but MacOS X
  glfwGetFramebufferSize(g_window, &g_windowWidth, &g_windowHeight);

  // Connect the callbacks for interactive control
  glfwSetWindowSizeCallback(g_window, windowSizeCallback);
  glfwSetKeyCallback(g_window, keyCallback);
  glfwSetCursorPosCallback(g_window, cursorPosCallback);
  glfwSetMouseButtonCallback(g_window, mouseButtonCallback);

  // --- Correction : initialiser la caméra avant le callback ---
  if (!g_cam) g_cam = std::make_shared<Camera>();
}

void initOpenGL()
{
  // Load extensions for modern OpenGL
  if(!gladLoadGL(glfwGetProcAddress)) {
    std::cerr << "[Failed to initialize OpenGL context]" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  // supported from OpenGL 4.3
  // glEnable(GL_DEBUG_OUTPUT);    // Modern error callback functionality
  // glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS); // For recovering the line where the error occurs, set a debugger breakpoint in DebugMessageCallback
  // glDebugMessageCallback(debugMessageCallback, 0); // Specifies the function to call when an error message is generated.

  glCullFace(GL_BACK); // Specifies the faces to cull (here the ones pointing away from the camera)
  glEnable(GL_CULL_FACE); // Enables face culling (based on the orientation defined by the CW/CCW enumeration).
  glDepthFunc(GL_LESS);   // Specify the depth test for the z-buffer
  glEnable(GL_DEPTH_TEST);      // Enable the z-buffer test in the rasterization
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // specify the background color, used any time the framebuffer is cleared

  // Loads and compile the programmable shader pipeline
  try {
    g_scene.mainShader = ShaderProgram::genBasicShaderProgram("src/vertexShader.glsl", "src/fragmentShader.glsl");
    g_scene.mainShader->stop();
  } catch(std::exception &e) {
    std::cerr << "[Error loading shader program] " << e.what() << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

void initScene()
{
  int width, height;
  glfwGetWindowSize(g_window, &width, &height);
  g_cam->setAspectRatio(static_cast<float>(width)/static_cast<float>(height));

  // Load meshes in the scene
  {
    g_scene.rigid = std::make_shared<Mesh>();
    g_scene.rigid->addSphere(g_ballRadius, 32, 32); // sphère de rayon global
    g_scene.rigid->init();

    // for the solver
    g_scene.rigidAtt = std::make_shared<Sphere>(g_ballRadius); // même rayon que la géométrie
    g_scene.rigidAtt->X = posInit;
    g_scene.solver.init(g_scene.rigidAtt.get());
    // Mémorise la position initiale de la balle pour la caméra libre
    g_ballInitPos = glm::vec3(posInit.x, posInit.y, posInit.z);

    g_scene.plane = std::make_shared<Mesh>();
    g_scene.plane->addPlane();
    g_scene.plane->init();
    g_scene.planeMat = glm::translate(glm::mat4(1.0), glm::vec3(0, 0, -2.5f)); // mur arrière
    // Agrandir le sol ici (par exemple 10x plus grand)
    g_scene.floorMat = glm::scale(glm::mat4(1.0), glm::vec3(40.0f, 1.0f, 40.0f)) *
      glm::translate(glm::mat4(1.0), glm::vec3(0, -1.0, 0)) *
      glm::rotate(glm::mat4(1.0), (float)(-0.5f*M_PI), glm::vec3(1.0, 0.0, 0.0));

    g_scene.table = std::make_shared<Mesh>();
    g_scene.table->addBox(1.37f, 0.02f, 2.74f); // dimensions standard pingpong (L, H, l)
    g_scene.table->init();
    g_scene.tableMat = glm::translate(glm::mat4(1.0), glm::vec3(0, -0.5f, -1.13f)); // aligné avec le mur au fond

    // filet : dimensions typiques (épaisseur 1cm, hauteur 10cm, longueur table)
    g_scene.net = std::make_shared<Mesh>();
    g_scene.net->addBox(1.37f, g_scene.netHeight, 0.01f); // L, H, épaisseur
    g_scene.net->init();
    g_scene.netMat = glm::translate(glm::mat4(1.0), glm::vec3(0, -0.5f + 0.02f/2 + g_scene.netHeight/2, -1.13f))
      * glm::translate(glm::mat4(1.0), glm::vec3(0, 0, 2.74f/2 - 2.74f/2));
  }

  // Load textures
  {
    g_albedoTex = loadTextureFromFileToGPU("data/dice.png");
    g_normalTex = loadTextureFromFileToGPU("data/normal.png");
  }

  // Setup textures on the GPU
  {
    g_albedoTexOnGPU = g_availableTextureSlot;
    ++g_availableTextureSlot;
    glActiveTexture(GL_TEXTURE0 + g_albedoTexOnGPU);
    glBindTexture(GL_TEXTURE_2D, g_albedoTex);
  }
  {
    g_normalTexOnGPU = g_availableTextureSlot;
    ++g_availableTextureSlot;
    glActiveTexture(GL_TEXTURE0 + g_normalTexOnGPU);
    glBindTexture(GL_TEXTURE_2D, g_normalTex);
  }

  // Setup light
  g_scene.light.position = glm::vec3(g_scene.rigidAtt->X.x, g_scene.rigidAtt->X.y, g_scene.rigidAtt->X.z); // la lumière suit la balle
  g_scene.light.color = sunColor; // couleur unifiée
  g_scene.light.intensity = 2.0f;

  // Adjust the camera to the mesh
  g_meshScale = g_scene.scene_radius;
  // Utilise la position initiale de la balle comme cible caméra libre
  glm::vec3 camTarget = g_cameraFollowsBall ? glm::vec3(g_scene.rigidAtt->X.x, g_scene.rigidAtt->X.y, g_scene.rigidAtt->X.z) : g_ballInitPos;
  if (g_cameraFollowsBall) {
    glm::vec3 camPos = camTarget + glm::vec3(0.0, 2.0, 10.0);
    g_cam->setPosition(camPos);
    glm::vec3 dir = glm::normalize(camTarget - camPos);
    float yaw = atan2(dir.x, dir.z); // Yaw autour de l'axe Y
    float pitch = asin(-dir.y);        // Pitch autour de l'axe X
    g_cam->setRotation(glm::vec3(pitch, yaw, 0.0f));
  } else {
    // Initialisation orbite
    g_camOrbitRadius = 6.0f;
    g_camOrbitYaw = 0.0f;
    g_camOrbitPitch = 0.0f;
    glm::vec3 camPos = camTarget + glm::vec3(0.0, 2.0, g_camOrbitRadius);
    g_cam->setPosition(camPos);
    g_cam->setRotation(glm::vec3(0.0f, 0.0f, 0.0f));
  }
  g_cam->setNear(g_meshScale/100.f);
  g_cam->setFar(6.0*g_meshScale);

  g_scene.rigidMat = g_scene.rigidAtt->worldMat();
}

void init()
{
  initGLFW();                   // Windowing system
  windowSizeCallback(g_window, g_windowWidth, g_windowHeight); // DÉPLACÉ ICI
  initOpenGL();                 // OpenGL Context and shader pipeline
  initScene();                  // Actual scene to render
  initFullscreenQuad();         // Quad plein écran pour ray tracing
}

void clear()
{
  g_cam.reset();
  g_scene.rigid.reset();
  g_scene.plane.reset();
  g_scene.mainShader.reset();
  // Libération des VAO/VBO du quad plein écran
  if (quadVAO) glDeleteVertexArrays(1, &quadVAO);
  if (quadVBO) glDeleteBuffers(1, &quadVBO);
  // Libération explicite des textures
  if (g_albedoTex) glDeleteTextures(1, &g_albedoTex);
  if (g_normalTex) glDeleteTextures(1, &g_normalTex);
  // Désactivation des callbacks GLFW
  if (g_window) {
    glfwSetWindowSizeCallback(g_window, nullptr);
    glfwSetKeyCallback(g_window, nullptr);
    glfwSetCursorPosCallback(g_window, nullptr);
    glfwSetMouseButtonCallback(g_window, nullptr);
  }
  glfwDestroyWindow(g_window);
  glfwTerminate();
  std::exit(EXIT_SUCCESS); // Contournement pour éviter le segfault à la fermeture
}

// The main rendering call
void render()
{
    // S'assurer que le viewport est bien défini
    glViewport(0, 0, g_windowWidth, g_windowHeight);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // Désactive le depth test pour le quad plein écran
    glDisable(GL_DEPTH_TEST);
    g_scene.mainShader->use();
    // Envoi des matrices et position caméra au shader
    g_scene.mainShader->set("camPos", g_cam->getPosition());
    g_scene.mainShader->set("viewMat", g_cam->computeViewMatrix());
    g_scene.mainShader->set("projMat", g_cam->computeProjectionMatrix());
    g_scene.mainShader->set("fov", g_cam->getFov());
    g_scene.mainShader->set("aspect", g_cam->getAspectRatio());
    // Envoi de la position de la balle au shader ray tracing
    g_scene.mainShader->set("sphereCenter", glm::vec3(g_scene.rigidAtt->X.x, g_scene.rigidAtt->X.y, g_scene.rigidAtt->X.z));
    // Envoi de la hauteur du filet au shader ray tracing
    g_scene.mainShader->set("netHeight", g_scene.netHeight);
    // Envoi du rayon de la sphère au shader ray tracing
    g_scene.mainShader->set("sphereRadius", g_ballRadius);
    // Envoi des uniforms pour la scène raytracée
    g_scene.mainShader->set("tableXmin", tableXmin);
    g_scene.mainShader->set("tableXmax", tableXmax);
    g_scene.mainShader->set("tableYmin", tableYmin);
    g_scene.mainShader->set("tableYmax", tableYmax);
    g_scene.mainShader->set("tableZmin", tableZmin);
    g_scene.mainShader->set("tableZmax", tableZmax);
    g_scene.mainShader->set("netWidth", netWidth);
    g_scene.mainShader->set("netHeight", netHeight);
    g_scene.mainShader->set("netThickness", netThickness);
    // Uniforms pour le sol raytracé
    float floorXmin = tableXmin - 5.0f;
    float floorXmax = tableXmax + 5.0f;
    float floorYmin = -1.0f;
    float floorYmax = -1.0f; // strictement plat
    float floorZmin = tableZmin - 5.0f;
    float floorZmax = tableZmax + 5.0f;
    g_scene.mainShader->set("floorXmin", floorXmin);
    g_scene.mainShader->set("floorXmax", floorXmax);
    g_scene.mainShader->set("floorYmin", floorYmin);
    g_scene.mainShader->set("floorYmax", floorYmax);
    g_scene.mainShader->set("floorZmin", floorZmin);
    g_scene.mainShader->set("floorZmax", floorZmax);
    // Correction dynamique de la hauteur ET de la position X du mur du fond (mur centré sur la balle)
    tReal xBall = g_scene.rigidAtt->X.x;
    float wallXcenter = (xBall > (tableXmin+tableXmax)/2) ? std::min(xBall, tableXmax+0.5f) : std::max(xBall, tableXmin-0.5f);

    float wallHalfLength = (wallLength) * 0.5f;
    float wallXmin_dyn = wallXcenter - wallHalfLength;
    float wallXmax_dyn = wallXcenter + wallHalfLength;
    g_scene.mainShader->set("wallXmin", wallXmin_dyn);
    g_scene.mainShader->set("wallXmax", wallXmax_dyn);
    // Hauteur dynamique (déjà présent)
    float wallYcenter = g_scene.rigidAtt->X.y;
    float wallHalfHeight = (wallHeight) * 0.5f;
    float wallYmin_dyn = wallYcenter - wallHalfHeight;
    float wallYmax_dyn = wallYcenter + wallHalfHeight;
    g_scene.mainShader->set("wallYmin", wallYmin_dyn);
    g_scene.mainShader->set("wallYmax", wallYmax_dyn);
    g_scene.mainShader->set("wallZmin", wallZmin);
    g_scene.mainShader->set("wallZmax", wallZmax);
    glBindVertexArray(quadVAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);
    g_scene.mainShader->stop();
    // Réactive le depth test pour la suite si besoin
    glEnable(GL_DEPTH_TEST);
}

// Update any accessible variable based on the current time
void update(const float currentTime)
{
  if(!g_appTimerStoppedP) {
    const float dt = currentTime - g_appTimerLastClockTime;
    g_appTimerLastClockTime = currentTime;
    g_appTimer += dt;
    g_scene.solver.step(std::min(dt, 0.017f));
    g_scene.rigidMat = g_scene.rigidAtt->worldMat();
  }
  g_scene.light.position = glm::vec3(g_scene.rigidAtt->X.x, g_scene.rigidAtt->X.y, g_scene.rigidAtt->X.z);
  if (g_cameraFollowsBall)
  {
      glm::vec3 camTarget = glm::vec3(
          g_scene.rigidAtt->X.x,
          g_scene.rigidAtt->X.y,
          g_scene.rigidAtt->X.z
      );
      glm::vec3 camPos = camTarget + glm::vec3(0.0, 1.0, 5.0);
      g_cam->setPosition(camPos);
      glm::vec3 dir = glm::normalize(camTarget - camPos);
      float yaw   = atan2(dir.x, dir.z);
      float pitch = asin(-dir.y);
      g_cam->setRotation(glm::vec3(pitch, yaw, 0.0f));
  }
  else
  {
      glm::vec3 target = g_ballInitPos;
      float camX = target.x + g_camOrbitRadius * cos(g_camOrbitPitch) * sin(g_camOrbitYaw);
      float camY = 1.5f;
      float camZ = target.z + g_camOrbitRadius * cos(g_camOrbitPitch) * cos(g_camOrbitYaw);
      glm::vec3 camPos(camX, camY, camZ);
      g_cam->setPosition(camPos);
      glm::vec3 dir = glm::normalize(target - camPos);
      float yaw   = atan2(dir.x, dir.z);
      float pitch = asin(-dir.y);
      g_cam->setRotation(glm::vec3(pitch, yaw, 0.0f));
  }

  // Rayon sphère visible
  static_cast<Sphere*>(g_scene.rigidAtt.get())->radius = g_ballRadius;
}

int main(int argc, char **argv)
{
  init();
  while(!glfwWindowShouldClose(g_window)) {
    update(static_cast<float>(glfwGetTime()));
    render();
    glfwSwapBuffers(g_window);
    glfwPollEvents();
  }
  clear();
  return EXIT_SUCCESS;
}
