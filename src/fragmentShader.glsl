#version 330 core

out vec4 FragColor;

in vec2 TexCoords;

uniform vec3 camPos;
uniform mat4 viewMat;
uniform mat4 projMat;
uniform vec3 sphereCenter;
uniform float sphereRadius;
uniform float fov;
uniform float aspect;

// Ray-sphere intersection
bool intersectSphere(vec3 ro, vec3 rd, vec3 center, float radius, out float t, out vec3 n) {
    vec3 oc = ro - center;
    float b = dot(oc, rd);
    float c = dot(oc, oc) - radius * radius;
    float h = b * b - c;
    if (h < 0.0) return false;
    h = sqrt(h);
    float t0 = -b - h;
    float t1 = -b + h;
    if (t0 > 0.0) t = t0;
    else if (t1 > 0.0) t = t1;
    else return false;
    vec3 p = ro + t * rd;
    n = normalize(p - center);
    return true;
}

// Ray-plane intersection (plan infini)
bool intersectPlane(vec3 ro, vec3 rd, vec3 p0, vec3 n, out float t) {
    float denom = dot(rd, n);
    if (abs(denom) < 1e-5) return false;
    t = dot(p0 - ro, n) / denom;
    return t > 0.0;
}

void main()
{
    vec2 ndc = TexCoords * 2.0 - 1.0;
    ndc.x *= aspect;
    float px = ndc.x * tan(radians(fov) * 0.5);
    float py = ndc.y * tan(radians(fov) * 0.5);
    vec3 ro = camPos;
    vec3 rd = normalize(vec3(px, py, -1.0)); // direction caméra OpenGL

    // Lumière
    vec3 lightPos = vec3(2.0, 2.0, 2.0);
    vec3 lightColor = vec3(1.0, 1.0, 0.8);
    float lightIntensity = 1.5;

    // Sphère (balle/soleil)
    vec3 sphereColor = vec3(0.95, 0.95, 0.85);

    // Sol (y = -1)
    vec3 floorP = vec3(0.0, -1.0, 0.0);
    vec3 floorN = vec3(0.0, 1.0, 0.0);
    vec3 floorColor = vec3(0.8, 0.8, 0.9);

    // Mur (z = -2.5)
    vec3 wallP = vec3(0.0, 0.0, -2.5);
    vec3 wallN = vec3(0.0, 0.0, 1.0);
    vec3 wallColor = vec3(0.36, 0.25, 0.20);

    // Table (box centrée en (0,-0.5,-1.13), L=1.37, H=0.02, l=2.74)
    vec3 tableMin = vec3(-1.37/2, -0.5-0.01, -1.13-2.74/2);
    vec3 tableMax = vec3( 1.37/2, -0.5+0.01, -1.13+2.74/2);
    vec3 tableColor = vec3(0.0, 0.35, 0.55);

    // Filet (box centrée en (0, -0.5+0.01+0.15/2, -1.13), L=1.37, H=0.15, l=0.01)
    vec3 netMin = vec3(-1.37/2, -0.5+0.01, -1.13-0.005);
    vec3 netMax = vec3( 1.37/2, -0.5+0.01+0.15, -1.13+0.005);
    vec3 netColor = vec3(0.9, 0.9, 0.9);

    float tMin = 1e20;
    vec3 hitColor = vec3(0.1, 0.1, 0.15);
    vec3 nHit = vec3(0.0);
    bool hit = false;

    // Sphère
    float tSph; vec3 nSph;
    if (intersectSphere(ro, rd, sphereCenter, sphereRadius, tSph, nSph) && tSph < tMin) {
        tMin = tSph;
        hitColor = sphereColor;
        nHit = nSph;
        hit = true;
    }
    // Sol
    float tFloor;
    if (intersectPlane(ro, rd, floorP, floorN, tFloor) && tFloor < tMin) {
        vec3 p = ro + tFloor * rd;
        // Limiter à une grande surface
        if (abs(p.x) < 2.0 && abs(p.z+1.13) < 2.0) {
            tMin = tFloor;
            hitColor = floorColor;
            nHit = floorN;
            hit = true;
        }
    }
    // Mur
    float tWall;
    if (intersectPlane(ro, rd, wallP, wallN, tWall) && tWall < tMin) {
        vec3 p = ro + tWall * rd;
        if (abs(p.x) < 2.0 && abs(p.y+0.5) < 2.0) {
            tMin = tWall;
            hitColor = wallColor;
            nHit = wallN;
            hit = true;
        }
    }
    // Table (AABB)
    float tNear = -1e20, tFar = 1e20;
    for (int i = 0; i < 3; ++i) {
        float invD = 1.0 / rd[i];
        float t0 = (tableMin[i] - ro[i]) * invD;
        float t1 = (tableMax[i] - ro[i]) * invD;
        if (invD < 0.0) { float tmp = t0; t0 = t1; t1 = tmp; }
        tNear = max(tNear, t0);
        tFar = min(tFar, t1);
    }
    if (tNear < tFar && tNear > 0.0 && tNear < tMin) {
        tMin = tNear;
        hitColor = tableColor;
        nHit = vec3(0,1,0);
        hit = true;
    }
    // Filet (AABB)
    tNear = -1e20; tFar = 1e20;
    for (int i = 0; i < 3; ++i) {
        float invD = 1.0 / rd[i];
        float t0 = (netMin[i] - ro[i]) * invD;
        float t1 = (netMax[i] - ro[i]) * invD;
        if (invD < 0.0) { float tmp = t0; t0 = t1; t1 = tmp; }
        tNear = max(tNear, t0);
        tFar = min(tFar, t1);
    }
    if (tNear < tFar && tNear > 0.0 && tNear < tMin) {
        tMin = tNear;
        hitColor = netColor;
        nHit = vec3(0,1,0);
        hit = true;
    }

    if (hit) {
        vec3 p = ro + tMin * rd;
        vec3 lightDir = normalize(lightPos - p);
        float diff = max(dot(nHit, lightDir), 0.0);
        vec3 color = hitColor * diff * lightIntensity;
        FragColor = vec4(color, 1.0);
    } else {
        FragColor = vec4(0.1, 0.1, 0.15, 1.0); // fond
    }
}