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
uniform float netHeight;

// Ray-sphere intersection
bool intersectSphere(vec3 ro, vec3 rd, vec3 center, float radius, out float t, out vec3 n) {
    vec3 oc = ro - center;
    float b = 2.0 * dot(oc, rd);
    float c = dot(oc, oc) - radius * radius;
    float h = b * b - 4.0 * c;
    if (h < 0.0) return false;
    h = sqrt(h);
    float t0 = (-b - h) * 0.5;
    float t1 = (-b + h) * 0.5;
    if (t0 > 0.0) t = t0;
    else if (t1 > 0.0) t = t1;
    else return false;
    vec3 p = ro + t * rd;
    n = normalize(p - center);
    return true;
}

// Ray-box intersection (AABB)
bool intersectBox(vec3 ro, vec3 rd, vec3 boxMin, vec3 boxMax, out float tNear) {
    vec3 t0 = (boxMin - ro) / rd;
    vec3 t1 = (boxMax - ro) / rd;
    vec3 tmin = min(t0, t1);
    vec3 tmax = max(t0, t1);
    float tN = max(max(tmin.x, tmin.y), tmin.z);
    float tF = min(min(tmax.x, tmax.y), tmax.z);
    tNear = tN;
    return tF >= tN && tF > 0.0;
}

void main()
{
    vec2 ndc = TexCoords * 2.0 - 1.0;
    ndc.x *= aspect;
    float px = ndc.x * tan(radians(fov) * 0.5);
    float py = ndc.y * tan(radians(fov) * 0.5);
    vec3 ro = camPos;
    vec3 rd = normalize(vec3(px, py, -1.0));

    float tSph; vec3 nSph;
    bool sphHit = intersectSphere(ro, rd, sphereCenter, sphereRadius, tSph, nSph);

    // Table (box)
    vec3 tableSize = vec3(1.37, 0.02, 2.74);
    vec3 tableCenter = vec3(0.0, -0.5, -1.13);
    vec3 tableMin = tableCenter - tableSize * 0.5;
    vec3 tableMax = tableCenter + tableSize * 0.5;
    float tTable;
    bool tableHit = intersectBox(ro, rd, tableMin, tableMax, tTable);

    // Filet (box)
    vec3 netSize = vec3(1.37, netHeight, 0.01);
    vec3 netCenter = vec3(0.0, -0.5 + 0.02/2.0 + netHeight/2.0, -1.13);
    vec3 netMin = netCenter - netSize * 0.5;
    vec3 netMax = netCenter + netSize * 0.5;
    float tNet;
    bool netHit = intersectBox(ro, rd, netMin, netMax, tNet);

    // Sol (box plate sous la table, dimensions visuelles proches du main.cpp)
    vec3 floorSize = vec3(4.0, 0.02, 4.0);
    vec3 floorCenter = vec3(0.0, -1.0, 0.0);
    vec3 floorMin = floorCenter - floorSize * 0.5;
    vec3 floorMax = floorCenter + floorSize * 0.5;
    float tFloor;
    bool floorHit = intersectBox(ro, rd, floorMin, floorMax, tFloor);

    // Mur du fond (box verticale derrière la table)
    vec3 wallSize = vec3(4.0, 2.0, 0.02);
    vec3 wallCenter = vec3(0.0, 0.0, -2.5);
    vec3 wallMin = wallCenter - wallSize * 0.5;
    vec3 wallMax = wallCenter + wallSize * 0.5;
    float tWall;
    bool wallHit = intersectBox(ro, rd, wallMin, wallMax, tWall);

    // On cherche l'objet le plus proche
    float tMin = 1e20;
    int hitType = 0;
    vec3 hitNormal = vec3(0);
    vec3 hitPos = vec3(0);
    vec3 baseColor = vec3(0.03, 0.03, 0.05); // fond sombre réduit
    if (floorHit && tFloor > 0.0 && tFloor < tMin) {
        tMin = tFloor;
        baseColor = vec3(0.8, 0.8, 0.9); // sol gris clair
        hitType = 1;
        hitPos = ro + tFloor * rd;
        hitNormal = vec3(0, 1, 0);
    }
    if (wallHit && tWall > 0.0 && tWall < tMin) {
        tMin = tWall;
        baseColor = vec3(0.36, 0.25, 0.20); // mur marron
        hitType = 2;
        hitPos = ro + tWall * rd;
        hitNormal = vec3(0, 0, 1);
    }
    if (tableHit && tTable > 0.0 && tTable < tMin) {
        tMin = tTable;
        baseColor = vec3(0.0, 0.35, 0.55); // table bleu
        hitType = 3;
        hitPos = ro + tTable * rd;
        hitNormal = vec3(0, 1, 0);
    }
    if (netHit && tNet > 0.0 && tNet < tMin) {
        tMin = tNet;
        baseColor = vec3(0.9, 0.9, 0.9); // filet blanc
        hitType = 4;
        hitPos = ro + tNet * rd;
        hitNormal = vec3(0, 1, 0);
    }
    if (sphHit && tSph > 0.0 && tSph < tMin) {
        tMin = tSph;
        baseColor = vec3(0.95, 0.95, 0.85); // balle lumineuse (soleil)
        hitType = 5;
        hitPos = ro + tSph * rd;
        hitNormal = nSph;
    }

    vec3 color = baseColor;
    // Si ce n'est pas la balle, on éclaire avec la balle
    if (hitType != 0 && hitType != 5) {
        vec3 lightDir = normalize(sphereCenter - hitPos);
        float dist = length(sphereCenter - hitPos);
        float tShadowSph, tShadowTable, tShadowNet, tShadowFloor, tShadowWall;
        vec3 nShadow;
        bool shadowSph = intersectSphere(hitPos + hitNormal * 0.001, lightDir, sphereCenter, sphereRadius, tShadowSph, nShadow);
        bool shadowTable = intersectBox(hitPos + hitNormal * 0.001, lightDir, tableMin, tableMax, tShadowTable);
        bool shadowNet = intersectBox(hitPos + hitNormal * 0.001, lightDir, netMin, netMax, tShadowNet);
        bool shadowFloor = intersectBox(hitPos + hitNormal * 0.001, lightDir, floorMin, floorMax, tShadowFloor);
        bool shadowWall = intersectBox(hitPos + hitNormal * 0.001, lightDir, wallMin, wallMax, tShadowWall);
        float intensity = 0.0;
        float diffuse = 0.0;
        // Si le point voit la balle et aucun autre objet n'est entre le point et la balle
        bool shadow = false;
        if (shadowSph && tShadowSph > 0.0 && tShadowSph < dist) {
            // Vérifie qu'aucun autre objet n'est plus proche que la balle
            shadow = ( (!shadowTable || tShadowTable < 0.0 || tShadowTable >= dist)
                    && (!shadowNet   || tShadowNet   < 0.0 || tShadowNet   >= dist)
                    && (!shadowFloor || tShadowFloor < 0.0 || tShadowFloor >= dist)
                    && (!shadowWall  || tShadowWall  < 0.0 || tShadowWall  >= dist) );
        }
        if (shadow) {
            intensity = 2.0 / (dist * dist); // ou autre valeur
            diffuse = max(dot(hitNormal, lightDir), 0.0);
        }
        color = baseColor * (diffuse * intensity);
    }
    FragColor = vec4(color, 1.0);
}