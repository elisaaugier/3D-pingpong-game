#include "RigidSolver.hpp"

// Initialisation de la variable globale posInit
Vec3f posInit((tableXmin + tableXmax)/2, tableYmax + 0.5f, tableZmax);

tReal speed = 2.0f;
tReal g = 9.81f;
tReal restitution = 0.561f;

const tReal sphereRadius = 0.04f;
const tReal tableHeight = 0.15f;
const tReal tableYmax = -0.5f;
const tReal tableYmin = tableYmax - tableHeight;
const tReal tableZmin = -1.13f - 3.5f/2.0f;
const tReal tableZmax = -1.13f + 3.5f/2.0f;
const tReal tableXmin = -2.0f;
const tReal tableXmax = 2.0f;
const tReal wallHeight = 0.20f;
const tReal wallWidth = 0.06f;
const tReal wallLength = 0.2f;
const tReal wallYmin = tableYmin - 0.5f;
const tReal wallYmax = tableYmax + 1.5f;
const tReal wallXmin = -2.5f;
const tReal wallXmax = wallXmin + wallLength;
const tReal wallZmax = tableZmin - 0.2f;
const tReal wallZmin = wallZmax - wallWidth;
const tReal netWidth = tableXmax - tableXmin;
const tReal netHeight = 0.15f;
const tReal netThickness = 0.01f;
const tReal netY = tableYmax + netHeight;
const tReal netZ = (tableZmin + tableZmax)/2.0f;

// (Si d'autres fonctions non-inline ou méthodes hors-classes sont nécessaires, les ajouter ici)
