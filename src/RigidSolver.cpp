#include "RigidSolver.hpp"

// Initialisation de la variable globale posInit
Vec3f posInit((tableXmin + tableXmax)/2, tableYmax + 0.5f, tableZmax);

tReal speed = 1.5f;
tReal g = 9.81f;
tReal restitution = 0.561f;
const tReal coef = 2.0f;
const tReal sphereRadius = 0.02f*coef;
const tReal tableHeight = 0.25f;
const tReal tableYmax = -0.5f;
const tReal tableYmin = tableYmax - tableHeight;
const tReal tableZmax = -1.13f + 3.5f/2.0f;
const tReal tableZmin = tableZmax-2.74f*coef;
const tReal tableXmin = -1.52f * coef/2;
const tReal tableXmax = 1.52f * coef/2;
const tReal wallHeight = 0.2f * coef;
const tReal wallWidth = 0.01f * coef;
const tReal wallLength = 0.2f*coef;
const tReal wallZmax = tableZmin - 0.2f;
const tReal wallZmin = wallZmax - wallWidth;
const tReal netWidth = tableXmax - tableXmin;
const tReal netHeight = 0.1f * coef;
const tReal netThickness = 0.01f;
const tReal netY = tableYmax + netHeight;
const tReal netZ = (tableZmin + tableZmax)/2.0f;

tReal wallXmin=(tableXmin+tableXmax)/2.0f;
tReal wallXmax=wallXmin + wallLength;
tReal wallYmin=tableYmax;
tReal wallYmax=wallYmin + wallHeight;


// (Si d'autres fonctions non-inline ou méthodes hors-classes sont nécessaires, les ajouter ici)
