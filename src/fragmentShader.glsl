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

// Uniforms for min/max variables
uniform float tableXmin, tableXmax, tableYmin, tableYmax, tableZmin, tableZmax;
uniform float wallXmin, wallXmax, wallYmin, wallYmax, wallZmin, wallZmax;
// NEW: circular paddle parameters (sent from main.cpp)
uniform float wallCenterX;
uniform float wallCenterY;
uniform float wallRadius;
uniform float netWidth, netHeight, netThickness;
// Floor min/max bounds
uniform float floorXmin, floorXmax, floorYmin, floorYmax, floorZmin, floorZmax;
// manual mode flag (1 = manual, 0 = automatic)
uniform int manualWallMode;
// --- AJOUT POUR SPOTS DE REBOND ---
#define MAX_BOUNCES 32
uniform int nTableBounces;
uniform vec3 tableBouncePositions[MAX_BOUNCES];
// --- FIN AJOUT ---

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

uniform int gameOver;

void main()
{
    /*if (gameOver == 1) {
        FragColor = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }*/

    vec2 ndc = TexCoords * 2.0 - 1.0;
    ndc.x *= aspect;
    float px = ndc.x * tan(radians(fov) * 0.5);
    float py = ndc.y * tan(radians(fov) * 0.5);
    vec3 ro = camPos;
    vec3 rd = normalize(vec3(px, py, -1.0));

    float tSph; vec3 nSph;
    bool sphHit = intersectSphere(ro, rd, sphereCenter, sphereRadius, tSph, nSph);

    // Table (box épaisse, flottante)
    vec3 tableMin = vec3(tableXmin, tableYmin, tableZmin);
    vec3 tableMax = vec3(tableXmax, tableYmax, tableZmax);
    float tTable;
    bool tableHit = intersectBox(ro, rd, tableMin, tableMax, tTable);

    // Filet (box)
    vec3 netMin = vec3(tableXmin, tableYmax, (tableZmax+tableZmin)/2-0.5*netThickness);
    vec3 netMax = vec3(tableXmax, tableYmax+netHeight, (tableZmax+tableZmin)/2+0.5*netThickness);
    float tNet;
    bool netHit = intersectBox(ro, rd, netMin, netMax, tNet);

    // Sol (box plate sous la table, dimensions visuelles proches du main.cpp)
    vec3 floorMin = vec3(floorXmin, floorYmin, floorZmin);
    vec3 floorMax = vec3(floorXmax, floorYmax, floorZmax);
    float tFloor;
    bool floorHit = intersectBox(ro, rd, floorMin, floorMax, tFloor);

    // Mur du fond (box verticale derrière la table)
    vec3 wallMin = vec3(wallXmin, wallYmin, wallZmin);
    vec3 wallMax = vec3(wallXmax, wallYmax, wallZmax);
    float tWall;
    bool wallHit = intersectBox(ro, rd, wallMin, wallMax, tWall);
    // paddle circle parameters
    float paddleCX = wallCenterX;
    float paddleCY = wallCenterY;
    float paddleR  = wallRadius;

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
        // compute impact point on wall
        vec3 wp = ro + tWall * rd;
        // check whether impact is inside circular paddle
        vec2 wp2 = wp.xz; // use x and y? wall plane is z=const so use x,y coordinates: use x and y
        // correct: we must use x and y components
        vec2 wp_xy = vec2(wp.x, wp.y);
        float d2 = dot(wp_xy - vec2(paddleCX, paddleCY), wp_xy - vec2(paddleCX, paddleCY));
        if (d2 <= paddleR * paddleR) {
            tMin = tWall;
            baseColor = (wallXmin >  (tableXmin+tableXmax)/2) ? vec3(0.0, 0.4, 1.0) : vec3(1.0, 0.2, 0.6); //couleur du mur du fond
            hitType = 2;
            hitPos = wp;
            hitNormal = vec3(0, 0, 1);
        }
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

    // --- AJOUT AFFICHAGE SPOTS ROUGES ---
    if (hitType == 3) { // sur la table
        float spotRadius = 0.06; // rayon visuel du spot
        for (int i = 0; i < nTableBounces; ++i) {
            float d = length(hitPos.xz - tableBouncePositions[i].xz);
            if (d < spotRadius) {
                baseColor = mix(baseColor, vec3(1.0, 0.0, 0.0), 0.85); // tâche rouge
            }
        }
    }
    // --- FIN AJOUT ---
    float spotAlpha = 0.0;

    vec3 color = baseColor;
    float ambientStrength = (manualWallMode == 1) ? 0.20 : 0.02; // intensité de la lumière ambiante dépend du mode manuel
    vec3 ambient = ambientStrength * baseColor;
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
        bool shadow = false;
        if (shadowSph && tShadowSph > 0.0 && tShadowSph < dist) {
            shadow = ( (!shadowTable || tShadowTable < 0.0 || tShadowTable >= dist)
                    && (!shadowNet   || tShadowNet   < 0.0 || tShadowNet   >= dist)
                    && (!shadowFloor || tShadowFloor < 0.0 || tShadowFloor >= dist)
                    && (!shadowWall  || tShadowWall  < 0.0 || tShadowWall  >= dist) );
        }
        if (shadow) {
            intensity = 2.0 / (dist * dist);
            diffuse = max(dot(hitNormal, lightDir), 0.0);
        }
        color = ambient + baseColor * (diffuse * intensity);
    } else {
        color = ambient + baseColor;
    }
    FragColor = vec4(color, 1.0);

    // dessin visuel de la raquette (disque) pour l'overlay
    // compute intersection of ray with plane z = wallZmax
    float tPaddle = (wallZmax - ro.z) / rd.z;
    if (tPaddle > 0.0) {
        vec3 pP = ro + tPaddle * rd;
        vec2 pPxy = vec2(pP.x, pP.y);
        if (length(pPxy - vec2(paddleCX, paddleCY)) <= paddleR) {
            // highlight paddle area
            baseColor = mix(baseColor, vec3(0.9,0.9,0.2), 0.45);
            hitType = 4; // visual paddle
            hitPos = pP;
            hitNormal = vec3(0,0,1);
        }
    }

    // draw a simple stick-figure beside the paddle (on the same wall plane)
    // helper: distance from point p to segment [a,b]
    float sdSegment(vec2 p, vec2 a, vec2 b) {
        vec2 pa = p - a;
        vec2 ba = b - a;
        float h = clamp(dot(pa, ba) / dot(ba, ba), 0.0, 1.0);
        return length(pa - ba * h);
    }

    float tStick = (wallZmax - ro.z) / rd.z;
    if (tStick > 0.0 && tStick < tMin) {
        vec3 pS = ro + tStick * rd;
        vec2 pSxy = vec2(pS.x, pS.y);
        // place stick figure to the right of the paddle by offset
        float sideOffset = paddleR + 0.25;
        vec2 stickCenter = vec2(paddleCX + sideOffset, paddleCY - 0.05);
        float stickScale = max(0.12, paddleR * 0.9);
        // head
        vec2 headC = stickCenter + vec2(0.0, 0.45 * stickScale);
        float headR = 0.18 * stickScale;
        float headDist = length(pSxy - headC);
        // body
        vec2 bodyTop = headC - vec2(0.0, headR);
        vec2 bodyBottom = bodyTop - vec2(0.0, 0.6 * stickScale);
        float bodyDist = sdSegment(pSxy, bodyTop, bodyBottom);
        // arms
        vec2 armLeft = bodyTop + vec2(-0.35 * stickScale, -0.15 * stickScale);
        vec2 armRight = bodyTop + vec2( 0.35 * stickScale, -0.15 * stickScale);
        float armLeftDist = sdSegment(pSxy, bodyTop - vec2(0.05*stickScale,0.0), armLeft);
        float armRightDist = sdSegment(pSxy, bodyTop - vec2(0.05*stickScale,0.0), armRight);
        // legs
        vec2 legLeft = bodyBottom + vec2(-0.25 * stickScale, -0.45 * stickScale);
        vec2 legRight = bodyBottom + vec2( 0.25 * stickScale, -0.45 * stickScale);
        float legLeftDist = sdSegment(pSxy, bodyBottom, legLeft);
        float legRightDist = sdSegment(pSxy, bodyBottom, legRight);
        float thickness = 0.035 * stickScale;
        bool hitStick = false;
        if (headDist <= headR) {
            baseColor = mix(baseColor, vec3(1.0,0.9,0.7), 0.9);
            hitStick = true;
        }
        if (bodyDist <= thickness) { baseColor = mix(baseColor, vec3(0.95,0.8,0.2), 0.9); hitStick = true; }
        if (armLeftDist <= thickness) { baseColor = mix(baseColor, vec3(0.95,0.8,0.2), 0.9); hitStick = true; }
        if (armRightDist <= thickness) { baseColor = mix(baseColor, vec3(0.95,0.8,0.2), 0.9); hitStick = true; }
        if (legLeftDist <= thickness) { baseColor = mix(baseColor, vec3(0.95,0.8,0.2), 0.9); hitStick = true; }
        if (legRightDist <= thickness) { baseColor = mix(baseColor, vec3(0.95,0.8,0.2), 0.9); hitStick = true; }
        if (hitStick) {
            hitType = 6; // stick figure
            hitPos = pS;
            hitNormal = vec3(0,0,1);
        }
    }
}