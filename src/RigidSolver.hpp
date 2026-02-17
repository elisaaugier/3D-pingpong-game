// ----------------------------------------------------------------------------
// RigidSolver.hpp
//
//  Created on: 18 Dec 2020
//      Author: Kiwon Um
//        Mail: kiwon.um@telecom-paris.fr
//
// Description: Simple Rigid Body Solver (DO NOT DISTRIBUTE!)
//
// Copyright 2020-2026 Kiwon Um
//
// The copyright to the computer program(s) herein is the property of Kiwon Um,
// Telecom Paris, France. The program(s) may be used and/or copied only with
// the written permission of Kiwon Um or in accordance with the terms and
// conditions stipulated in the agreement/contract under which the program(s)
// have been supplied.
// ----------------------------------------------------------------------------

#ifndef _RIGIDSOLVER_HPP_
#define _RIGIDSOLVER_HPP_

#include <glm/ext/matrix_transform.hpp>

#include "Vector3.hpp"
#include "Matrix3x3.hpp"

Vec3f posInit(0, 0, 0);

// Simple quaternion (w + xi + yj + zk)
struct Quaternion {
  tReal w, x, y, z;
  Quaternion(): w(1), x(0), y(0), z(0) {}
  Quaternion(tReal _w, tReal _x, tReal _y, tReal _z): w(_w), x(_x), y(_y), z(_z) {}

  static Quaternion identity() { return Quaternion(1,0,0,0); }

  Quaternion operator*(const Quaternion &r) const {
    return Quaternion(
      w*r.w - x*r.x - y*r.y - z*r.z,
      w*r.x + x*r.w + y*r.z - z*r.y,
      w*r.y - x*r.z + y*r.w + z*r.x,
      w*r.z + x*r.y - y*r.x + z*r.w);
  }

  void normalize() {
    const tReal n = std::sqrt(w*w + x*x + y*y + z*z);
    if(n>0) { const tReal inv = 1.0f/n; w*=inv; x*=inv; y*=inv; z*=inv; }
  }

  // build rotation matrix (rows r, cols c)
  Mat3f toMatrix() const {
    const tReal xx = x*x, yy = y*y, zz = z*z;
    const tReal xy = x*y, xz = x*z, yz = y*z;
    const tReal wx = w*x, wy = w*y, wz = w*z;
    return Mat3f(
      1 - 2*(yy+zz),  2*(xy - wz),    2*(xz + wy),
      2*(xy + wz),    1 - 2*(xx+zz),  2*(yz - wx),
      2*(xz - wy),    2*(yz + wx),    1 - 2*(xx+yy));
  }

  static Quaternion fromAxisAngle(const Vec3f &axis, const tReal angle) {
    const tReal h = angle * 0.5f;
    const tReal s = std::sin(h);
    return Quaternion(std::cos(h), axis.x*s, axis.y*s, axis.z*s);
  }
};

struct BodyAttributes {
  BodyAttributes() :
    X(posInit), R(Mat3f::I()), P(0, 0, 0), L(0, 0, 0),
    V(0, 0, 0), omega(0, 0, 0), F(0, 0, 0), tau(0, 0, 0), q(Quaternion::identity()) {}

  glm::mat4 worldMat() const
  {
    return glm::mat4(           // column-major
      R(0,0), R(1,0), R(2,0), 0,
      R(0,1), R(1,1), R(2,1), 0,
      R(0,2), R(1,2), R(2,2), 0,
      X[0],   X[1],   X[2],   1);
  }

  tReal M;                      // mass
  Mat3f I0, I0inv;              // inertia tensor and its inverse in body space
  Mat3f Iinv;                   // inverse of inertia tensor

  // rigid body state
  Vec3f X;                      // position
  Mat3f R;                      // rotation
  Quaternion q;                 // rotation quaternion (keeps R orthonormal)
  Vec3f P;                      // linear momentum
  Vec3f L;                      // angular momentum

  // auxiliary quantities
  Vec3f V;                      // linear velocity
  Vec3f omega;                  // angular velocity

  // force and torque
  Vec3f F;                      // force
  Vec3f tau;                    // torque

  // mesh's vertices in body space
  std::vector<Vec3f> vdata0;
};

class Sphere : public BodyAttributes {
public:
  explicit Sphere(const tReal r=0.1, const tReal dens=2.7, const Vec3f v0=Vec3f(0,0,0)) : radius(r) {
    V = v0;
    // Masse et inertie pour une sphère pleine
    M = dens * (4.0/3.0) * M_PI * radius * radius * radius;
    tReal I = (2.0/5.0) * M * radius * radius;
    I0 = Mat3f(I,0,0, 0,I,0, 0,0,I);
    I0inv = I0.inverse();
    Iinv = R * I0inv * R.transposed();
    // Un seul point pour la sphère (le centre)
    vdata0.push_back(Vec3f(0,0,0));
  }
  tReal radius;
};


class Box : public BodyAttributes {
public:
  explicit Box(
    const tReal w=1.0, const tReal h=1.0, const tReal d=1.0, const tReal dens=25.0,
    const Vec3f v0=Vec3f(0, 0, 0), const Vec3f omega0=Vec3f(0, 0, 0)) :
    width(w), height(h), depth(d)
  {
    V = v0;                     // initial velocity
    omega = omega0;             // initial angular velocity

    // mass and inertia (body-space) initialization
    M = dens * width * height * depth;
    {
      const tReal ix = (1.0/12.0) * M * (height*height + depth*depth);
      const tReal iy = (1.0/12.0) * M * (width*width + depth*depth);
      const tReal iz = (1.0/12.0) * M * (width*width + height*height);
      I0 = Mat3f(ix, 0, 0,  0, iy, 0,  0, 0, iz);
      I0inv = I0.inverse();
      Iinv = R * I0inv * R.transposed();
    }

    // vertices data (8 vertices)
    vdata0.push_back(Vec3f(-0.5*w, -0.5*h, -0.5*d));
    vdata0.push_back(Vec3f( 0.5*w, -0.5*h, -0.5*d));
    vdata0.push_back(Vec3f( 0.5*w,  0.5*h, -0.5*d));
    vdata0.push_back(Vec3f(-0.5*w,  0.5*h, -0.5*d));

    vdata0.push_back(Vec3f(-0.5*w, -0.5*h,  0.5*d));
    vdata0.push_back(Vec3f( 0.5*w, -0.5*h,  0.5*d));
    vdata0.push_back(Vec3f( 0.5*w,  0.5*h,  0.5*d));
    vdata0.push_back(Vec3f(-0.5*w,  0.5*h,  0.5*d));
  }

  // rigid body property
  tReal width, height, depth;
};

class RigidSolver {
public:
  explicit RigidSolver(
    BodyAttributes *body0=nullptr, const Vec3f g=Vec3f(0, 0, 0)) :
    body(body0), _g(g), _step(0), _sim_t(0) {}

  void init(BodyAttributes *body0)
  {
    body = body0;
    _step = 0;
    _sim_t = 0;

    if(body) {
      // initialize linear and angular momenta and world-space inverse inertia
      body->P = body->M * body->V;
      body->Iinv = body->R * body->I0inv * body->R.transposed();
      Mat3f Iworld = body->R * body->I0 * body->R.transposed();
      body->L = Iworld * body->omega;
    }
  }

  void step(const tReal dt)
  {
    std::cout << "t=" << _sim_t << " (dt=" << dt << ")" << std::endl;

    computeForceAndTorque();

    if(body) {
      // integrate linear momentum (semi-implicit Euler)
      body->P += body->F * dt;           // P_{n+1} = P_n + F * dt
      body->V = body->P / body->M;      // V = P / M
      body->X += body->V * dt;          // X_{n+1} = X_n + V_{n+1} * dt

      // --- Angular integration ---
      // Update angular momentum using resultant torque
      body->L += body->tau * dt;
      // Update angular velocity: omega = Iinv * L
      body->omega = body->Iinv * body->L;

      // Quaternion integration
      {
        const Quaternion q_delta = Quaternion::fromAxisAngle(body->omega, dt);
        body->q = (q_delta * body->q);
        body->q.normalize();
      }

      // Update rotation matrix from quaternion
      body->R = body->q.toMatrix();

      // Recompute world-space inverse inertia after updating R
      body->Iinv = body->R * body->I0inv * body->R.transposed();
    }

    ++_step;
    _sim_t += dt;
  }

  BodyAttributes *body;

private:
  void computeForceAndTorque()
  {
    if(!body) return;

    // gravity
    body->F = body->M * _g;

    // reset torque
    body->tau = Vec3f(0,0,0);

    // Ground contact detection and collision impulse
    const tReal groundY = -1.0f;
    const tReal groundSize = 1.1f; // demi-taille du sol (zone colorée)
    tReal maxPenetration = 0.0f;
    // Correction : tester le point le plus bas de la sphère
    tReal bottomY = body->X.y - static_cast<const Sphere*>(body)->radius;
    // --- Rebond sur le sol ---
    if(bottomY < groundY) {
      tReal penetration = groundY - bottomY;
      if(penetration > maxPenetration) maxPenetration = penetration;
      // Faire rebondir la balle avec un coefficient de restitution
      const float restitution = 1.f; // 0 = pas de rebond, 1 = rebond parfait
      if(body->V.y < 0) {
        body->V.y *= -restitution;
        body->P.y = body->V.y * body->M; // synchroniser l'impulsion
      }
    }
    // --- Rebond sur la table ---
    const tReal tableY = -0.5f + 0.02f/2.0f; // y du dessus de la table
    const tReal tableZmin = -1.13f - 2.74f/2.0f;
    const tReal tableZmax = -1.13f + 2.74f/2.0f;
    const tReal tableXmin = -1.37f/2.0f;
    const tReal tableXmax = 1.37f/2.0f;
    // Si la balle est au-dessus de la table (en x/z) et touche le dessus
    if(body->X.x > tableXmin && body->X.x < tableXmax &&
       body->X.z > tableZmin && body->X.z < tableZmax &&
       bottomY < tableY && body->X.y > groundY + 0.05f) // éviter double rebond sol/table
    {
      tReal penetration = tableY - bottomY;
      if(penetration > maxPenetration) maxPenetration = penetration;
      const float restitution = 1.f;
      if(body->V.y < 0) {
        body->V.y *= -restitution;
        body->P.y = body->V.y * body->M;
      }
    }
    // Correction de position unique (max penetration)
    if(maxPenetration > 0) {
      body->X.y += maxPenetration;
    }

    // instantaneous external force on the 0th vertex at step 1
    if(_step==1 && !body->vdata0.empty()) {
      const Vec3f instF(0.0f, 0.0f, 0.0f);
      // application point in world coordinates (relative to center): R * v0
      const Vec3f r_world = body->R * body->vdata0[0];
      body->F += instF;                  // add to resultant force
      body->tau += r_world.crossProduct(instF); // add torque = r x F
    }
  } 
  // simulation parameters
  Vec3f _g;                     // gravity
  tIndex _step;                 // step count
  tReal _sim_t;                 // simulation time
};

#endif  /* _RIGIDSOLVER_HPP_ */