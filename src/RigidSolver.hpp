#ifndef _RIGIDSOLVER_HPP_
#define _RIGIDSOLVER_HPP_

#include <glm/ext/matrix_transform.hpp>
#include <iostream>
#include <ostream>
#include <vector>

#include "Vector3.hpp"
#include "Matrix3x3.hpp"


extern const tReal sphereRadius; 
extern const tReal tableHeight,tableYmax,tableYmin,tableZmin,tableZmax,tableXmin,tableXmax;
extern const tReal wallHeight,wallWidth,wallLength,wallYmin,wallYmax,wallXmin,wallXmax,wallZmax,wallZmin;
extern const tReal netWidth,netHeight,netThickness,netY,netZ;

extern Vec3f posInit;
extern tReal speed;
extern tReal g;
extern tReal restitution;


// Quaternion (w + xi + yj + zk)
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
  explicit Sphere(const tReal r=0.1, const tReal dens=11.2f, const Vec3f v0=Vec3f(0,0,0)) : radius(r) {
    V = v0;
    // Masse et inertie pour une sphère pleine
    M = 0.003f;
    tReal I = (2.0/5.0) * M * radius * radius;
    I0 = Mat3f(I,0,0, 0,I,0, 0,0,I);
    I0inv = I0.inverse();
    Iinv = R * I0inv * R.transposed();
    // Un seul point pour la sphère (le centre)
    vdata0.push_back(Vec3f(0,0,0));
  }
  tReal radius;
  // Intersection segment-sphère (renvoie true si le segment [orig, end] traverse la sphère)
  bool intersectSegment(const glm::vec3& orig, const glm::vec3& end) const {
    glm::vec3 center(X.x, X.y, X.z);
    glm::vec3 d = end - orig;
    glm::vec3 oc = orig - center;
    float a = glm::dot(d, d);
    float b = 2.0f * glm::dot(oc, d);
    float c = glm::dot(oc, oc) - radius * radius;
    float disc = b * b - 4 * a * c;
    if (disc < 0) return false;
    float sqrtDisc = sqrt(disc);
    float t1 = (-b - sqrtDisc) / (2.0f * a);
    float t2 = (-b + sqrtDisc) / (2.0f * a);
    // Intersection si t dans [0,1]
    return (t1 >= 0.0f && t1 <= 1.0f) || (t2 >= 0.0f && t2 <= 1.0f);
  }
  // Détecte la collision avec un plan (point sur le plan et normale)
  // Retourne true si collision, et remplit penetration et normalContact
  bool checkPlaneCollision(const Vector3<tReal>& planePoint, const Vector3<tReal>& planeNormal, tReal& penetration, Vector3<tReal>& normalContact) const {
      Vector3<tReal> center(X.x, X.y, X.z);
      tReal dist = (center - planePoint).dotProduct(planeNormal);
      if (dist < radius) {
          penetration = radius - dist;
          normalContact = planeNormal;
          return true;
      }
      return false;
  }

  // Calcule la vitesse initiale V0 pour aller de A à B en passant par dessus le filet
  // g doit être positif (ex: 9.81)

  static Vector3<float> computeShotWithNetShortestTime(
    const Vector3<float>& A,
    const Vector3<float>& B)
{
    auto heightAtNet = [&](float T)
    {
        Vector3<float> v0 = (B - A) / T;
        v0.y += 0.5f * g * T;

        if (fabs(v0.z) < 1e-6f)
            return -1e-6f;

        float tNet = (netZ - A.z) / v0.z;

        if (tNet <= 0.0f || tNet >= T)
            return -1e-6f;

        float yAtNet =
            A.y + v0.y * tNet
            - 0.5f * g * tNet * tNet;

        return yAtNet;
    };

    float Tmin = 0.05f;
    float Tmax = 2.0f;

    for (int i = 0; i < 40; i++)
    {
        float Tmid = 0.5f * (Tmin + Tmax);

        if (heightAtNet(Tmid) > netY + 10*sphereRadius)
            Tmax = Tmid;
        else
            Tmin = Tmid;
    }

    float Tfinal = Tmax;

    Vector3<float> v0 = (B - A) / Tfinal;
    v0.y += 0.5f * g * Tfinal;

    return v0;
}



  static Vector3<float> computeMinimalShotWithNet(const Vector3<float>& A,const Vector3<float>& B)
  {
    const float eps = 1e-6f;
    const float step = 0.001f;     // incrément vitesse pour recherche
    const float maxFactor = 3.0f;  // limite de vitesse

    Vector3<float> d = B - A;
    float dxz = std::sqrt(d.x*d.x + d.z*d.z);
    float dy  = d.y;

    if (dxz < eps) return Vector3<float>(0,0,0);

    // vitesse minimale théorique sans filet (solution basse)
    float vMin0 = std::sqrt(g*dxz*dxz / (2.0f*(dxz * 0.0f - dy + eps))); 
    // 0.0f pour tanTheta minimale → solution basse
    // ajoute eps pour éviter div par 0

    float v = vMin0;

    while (v < vMin0 * maxFactor)
    {
        float v2 = v*v;
        float v4 = v2*v2;
        float disc = v4 - g*(g*dxz*dxz + 2.0f*dy*v2);

        if (disc >= 0.0f)
        {
            float sqrtDisc = std::sqrt(disc);

            // ---- solution basse (tir le plus plat) ----
            float tanTheta = (v2 - sqrtDisc) / (g * dxz);

            float cosTheta = 1.0f / std::sqrt(1.0f + tanTheta*tanTheta);
            float sinTheta = tanTheta * cosTheta;

            Vector3<float> dirXZ(d.x, 0, d.z);
            dirXZ.normalize();

            Vector3<float> v0;
            v0.x = v * cosTheta * dirXZ.x;
            v0.z = v * cosTheta * dirXZ.z;
            v0.y = v * sinTheta;

            // ---- TEST PASSAGE FILET ----
            if (std::abs(v0.z) > eps)
            {
                float tNet = (netZ - A.z) / v0.z;
                if (tNet > 0.0f)
                {
                    float yAtNet = A.y + v0.y * tNet - 0.5f * g * tNet * tNet;

                    if (yAtNet > netY+30*sphereRadius)
                    {
                        // vitesse minimale réalisable
                        return v0;
                    }
                }
            }
        }

        v += step;  // augmente vitesse si filet non franchi
    }

    // fallback si aucune solution trouvée
    return Vector3<float>(0,0,0);
  }

  static Vector3<float> computeR1FromR2(
      const Vector3<float>& A,
      const Vector3<float>& R2,
      float zR1)
  {
      float dz = R2.z - A.z;

      if (fabs(dz) < 1e-6f)
          return A; // cas dégénéré

      float t = (zR1 - A.z) / dz;
      Vector3<tReal> R1;
      R1.x = A.x + t * (R2.x - A.x);
      R1.y = R2.y;
      R1.z = zR1;

      return R1;
  }

  static Vector3<tReal> computeService(const Vector3<tReal>& A, const Vector3<tReal>& R2)
  {
    // R1 dans la diagonale entre A et R2
    tReal zR1 = netZ + (tableZmax-tableZmin) * 0.25f;

    Vector3<tReal> R1 = computeR1FromR2(A, R2, zR1);
    float dz = R2.z - A.z;
    if (fabs(dz) < 1e-6f)
    {
        return Vector3<float>(0,0,0);
    }

    Vector3<tReal> V1 = computeShotWithNetShortestTime(R1,R2); //vitesse nécessaire post rebond
    std::cout << "V1 theorique = (" << V1.x << ", " << V1.y << ", " << V1.z << ")" << std::endl;
    Vector3<tReal> V1before(V1.x, -1 * V1.y / restitution, V1.z); // vitesse que doit avoir la balle avant le rebond en R1 pour atteindre R2 après rebond
    std::cout << "V1before theorique = (" << V1before.x << ", " << V1before.y << ", " << V1before.z << ")" << std::endl;
    Vector3<tReal> V0 = computeV0GivenV1(A,R1,V1before); //vitesse nécessaire pour atteindre R1
    return V0;
  }

  static Vector3<float> computeV0GivenV1(
    const Vector3<float>& A,
    const Vector3<float>& R1,
    const Vector3<float>& V1before)
{
    Vector3<float> V0;

    // Horizontal inchangé
    V0.x = V1before.x;
    V0.z = V1before.z;

    // Vertical via conservation énergie
    float deltaY = R1.y - A.y;

    float inside = V1before.y * V1before.y + 2.0f * g * deltaY;

    if (inside < 0.0f)
        inside = 0.0f;

    V0.y = -std::sqrt(inside);

    return V0;
  } 

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
  // Intersection rayon-AABB (renvoie true si intersection)
  // Ajout d'un paramètre minDist pour ignorer les auto-intersections proches
  bool intersectRay(const glm::vec3& rayOrig, const glm::vec3& rayDir, float minDist = 1e-2f) const {
    glm::vec3 boxMin(X.x - 0.5f*width, X.y - 0.5f*height, X.z - 0.5f*depth);
    glm::vec3 boxMax(X.x + 0.5f*width, X.y + 0.5f*height, X.z + 0.5f*depth);
    float tNear = -1e30f, tFar = 1e30f;
    for (int i = 0; i < 3; ++i) {
      if (fabs(rayDir[i]) < 1e-6) {
        if (rayOrig[i] < boxMin[i] || rayOrig[i] > boxMax[i]) return false;
      } else {
        float t1 = (boxMin[i] - rayOrig[i]) / rayDir[i];
        float t2 = (boxMax[i] - rayOrig[i]) / rayDir[i];
        if (t1 > t2) std::swap(t1, t2);
        tNear = std::max(tNear, t1);
        tFar = std::min(tFar, t2);
        if (tNear > tFar) return false;
      }
    }
    // On ne considère l'intersection que si tNear > minDist
    return tFar > 0.0f && tNear > minDist;
  }
  // Intersection segment-AABB (renvoie true si le segment [orig, end] traverse la box)
  bool intersectSegment(const glm::vec3& orig, const glm::vec3& end) const {
    glm::vec3 boxMin(X.x - 0.5f*width, X.y - 0.5f*height, X.z - 0.5f*depth);
    glm::vec3 boxMax(X.x + 0.5f*width, X.y + 0.5f*height, X.z + 0.5f*depth);
    glm::vec3 d = end - orig;
    float tNear = 0.0f, tFar = 1.0f;
    for (int i = 0; i < 3; ++i) {
      if (fabs(d[i]) < 1e-6) {
        if (orig[i] < boxMin[i] || orig[i] > boxMax[i]) return false;
      } else {
        float t1 = (boxMin[i] - orig[i]) / d[i];
        float t2 = (boxMax[i] - orig[i]) / d[i];
        if (t1 > t2) std::swap(t1, t2);
        tNear = std::max(tNear, t1);
        tFar = std::min(tFar, t2);
        if (tNear > tFar) return false;
      }
    }
    return tFar >= 0.0f && tNear <= 1.0f && tNear <= tFar;
  }
};

class RigidSolver {
public:
  explicit RigidSolver(
    BodyAttributes *body0=nullptr, const Vec3f g=Vec3f(0, 0, 0)) :
    body(body0), _g(g), _step(0), _sim_t(0), gameOver(false), lastWallTouched(0) {}

  bool gameOver = false;       // true si la balle a touché le sol
  int lastWallTouched = 0;     // 0 = none 1 = mur du fond, 2 = mur de devant

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
    //std::cout << "t=" << _sim_t << " (dt=" << dt << ")" << std::endl;

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
  // Ajout pour gestion d'attente
  bool waitingAtR1 = false;
  float waitStartTime = 0.0f;
  Vector3<tReal> waitTargetB;

  void computeForceAndTorque()
  {
    if(!body) return;
    if (gameOver) {
      body->F = Vec3f(0,0,0);
      body->V = Vec3f(0,0,0);
      body->P = Vec3f(0,0,0);
      return;
    }

    // gravity
    body->F = body->M * _g;
    // reset torque
    body->tau = Vec3f(0,0,0);

    // --- Collision planaires unifiées ---
    tReal penetration;
    Vector3<tReal> normalContact;
    const Sphere* sphere = static_cast<const Sphere*>(body);

    // Sol (y = -1)
    if(sphere->checkPlaneCollision(Vector3<tReal>(0,-1,0), Vector3<tReal>(0,1,0), penetration, normalContact)) {
      if(body->V.y < 0) {
        gameOver = true;
        body->V = Vec3f(0,0,0);
        body->P = body->V * body->M;
        body->F = Vec3f(0,0,0);
        body->tau = Vec3f(0,0,0);
        body->X.y = -1.0f + sphere->radius;
      }
    }
    /*if(sphere->checkPlaneCollision(Vector3<tReal>(0,-1,0), Vector3<tReal>(0,1,0), penetration, normalContact)) {
      if(body->V.y < 0) {
        body->V -= (1.f + restitution) * body->V.dotProduct(normalContact) * normalContact;
        body->P = body->V * body->M;
        body->X.y = -1.0f + sphere->radius;
      }
    }*/ 
    // Table (dessus)
    if(sphere->checkPlaneCollision(Vector3<tReal>(0, tableYmax, 0), Vector3<tReal>(0,1,0), penetration, normalContact)) {
      if(body->X.x > tableXmin && body->X.x < tableXmax && body->X.z > tableZmin && body->X.z < tableZmax && body->X.y > tableYmin) {
        if(body->V.y < 0) {
          std::cout << "V avant rebond = (" << body->V.x << ", " << body->V.y << ", " << body->V.z << ")" << std::endl;
          body->V -= (1.f + restitution) * body->V.dotProduct(normalContact) * normalContact;
          body->P = body->V * body->M;
          body->X.y = tableYmax + sphere->radius;
          std::cout << "V1 reel apres rebond = (" << body->V.x << ", " << body->V.y << ", " << body->V.z << ")" << std::endl;
        }
      }
    }
    // Mur (arrière, plan z = tableZmin - 0.2)
    // On centre dynamiquement le mur sur la balle 
    tReal xBall = body->X.x;
    tReal wallXcenter = (xBall > (tableXmin+tableXmax)/2) ? std::min(xBall, tableXmax+0.5f) : std::max(xBall, tableXmin-0.5f);

    tReal wallHalfLength = (wallLength) * 0.5f;
    tReal wallInfX = wallXcenter - wallHalfLength;
    tReal wallMajX = wallXcenter + wallHalfLength;
    tReal wallY = body->X.y;
    tReal wallInfY = wallY - wallHeight/2;
    tReal wallMajY = wallY + wallHeight/2;
    if(sphere->checkPlaneCollision(Vector3<tReal>(0,0,wallZmax), Vector3<tReal>(0,0,1), penetration, normalContact)) {
      if(body->X.x > wallInfX && body->X.x < wallMajX && body->X.y > wallInfY && body->X.y < wallMajY) {
        if(body->V.z < 0) {
          Vector3<tReal> A(body->X.x, body->X.y, body->X.z);
          // Calcul direction balistique pour atteindre la cible
          tReal filetZ = netZ;
          tReal fondZ = tableZmax;
          tReal targetZ = (filetZ + fondZ) / 2.0f; // milieu Z entre filet et fond
          tReal targetX = static_cast<tReal>(rand()) / static_cast<tReal>(RAND_MAX) * (tableXmax - tableXmin) + tableXmin; // cible aléatoire en X
          Vector3<tReal> B(targetX, tableYmax, targetZ); // cible aléatoire sur le fond
          Vector3<tReal> V0 = Sphere::computeShotWithNetShortestTime(A,B);
          body->V = V0;
          body->P = body->V * body->M;
          body->X.z = tableZmin - 0.2f + sphere->radius;
          lastWallTouched = 1; // mur du fond
        }
      }
    }

    // rebond devant raquette (plan z = tableZmax + 0.2)
    if(sphere->checkPlaneCollision(Vector3<tReal>(0,0,tableZmax + 0.2f), Vector3<tReal>(0,0,-1), penetration, normalContact)) {
      // Limites identiques au mur visuel
      tReal minX = tableXmin - 0.2f;
      tReal maxX = tableXmax + 0.2f;
      tReal minY = tableYmin - 0.1f;
      tReal maxY = tableYmax + 2.0f;
      if(body->X.x > minX && body->X.x < maxX && body->X.y > minY && body->X.y < maxY) {
        if(body->V.z > 0) {
          Vector3<tReal> A(body->X.x, body->X.y, body->X.z);
          // Calcul direction balistique pour atteindre la cible
          tReal filetZ = netZ;
          tReal fondZ = tableZmin;
          tReal targetZ = (filetZ + fondZ) / 2.0f; // milieu Z entre filet et fond
          tReal targetX = static_cast<tReal>(rand()) / static_cast<tReal>(RAND_MAX) * (tableXmax - tableXmin) + tableXmin; // cible aléatoire en X
          Vector3<tReal> B(targetX, tableYmax, targetZ); // cible aléatoire sur le fond
          Vector3<tReal> V0 = Sphere::computeShotWithNetShortestTime(A,B);
          body->V = V0;
          body->P = body->V * body->M;
          lastWallTouched = 2; // mur de devant
        }
      }
    }

    // instantaneous external force on the 0th vertex at step 1
    if(_step==1 && !body->vdata0.empty()) {
      // Point de départ (A) : position actuelle de la balle
      Vector3<tReal> A(body->X.x, body->X.y, body->X.z);
      // Point d'arrivée (B)
      tReal filetZ = netZ;
      tReal fondZ = tableZmin;
      tReal targetZ = (filetZ + fondZ) / 2.0f; // milieu Z entre filet et fond
      tReal targetX = static_cast<tReal>(rand()) / static_cast<tReal>(RAND_MAX) * (tableXmax - tableXmin) + tableXmin; // cible aléatoire en X
      Vector3<tReal> B(targetX, tableYmax, targetZ); // cible aléatoire sur le fond
      //Vector3<tReal> V0 = Sphere::computeShotWithNetShortestTime(A,B);
      Vector3<tReal> V0 = Sphere::computeService(A,B);
      body->V = V0;
      body->P = body->V * body->M;
      // Optionnel : afficher la vitesse calculée pour debug
      std::cout << "V0 = (" << V0.x << ", " << V0.y << ", " << V0.z << ")" << std::endl;

    }
  } 
  // simulation parameters
  Vec3f _g;                     // gravity
  tIndex _step;                 // step count
  tReal _sim_t;                 // simulation time
};

#endif  /* _RIGIDSOLVER_HPP_ */