#version 330 core

out vec4 FragColor;

in vec2 TexCoords;

uniform vec3 camPos;
uniform mat4 viewMat;
uniform mat4 projMat;
uniform vec3 sphereCenter;
uniform float sphereRadius;
// velocity of the sphere (for trail rendering)
uniform vec3 sphereVelocity;
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
// trail uniforms
#define MAX_TRAIL 64
uniform int nTrail;
uniform vec3 trailPositions[MAX_TRAIL];
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

    // lighting: start from baseColor and compute a modest ambient term
    vec3 color = baseColor;
    float ambientStrength = (manualWallMode == 1) ? 0.06 : 0.015; // reduced ambient for less global brightness
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
    // Use the lighting result computed above (includes ambient and direct term)
    vec3 finalColor = color; // keep computed lighting (not a raw baseColor recomposition)

    // --- subtle white trail behind the ball (contrail-like) ---
    float vlen = length(sphereVelocity);
    if (vlen > 1e-4) {
        vec3 velDir = normalize(sphereVelocity);
        // trail parameters (adjustable)
        float trailLen = 0.6;                    // length of the contrail in world units (halved)
        float trailStartOffset = 0.05;           // start just behind the ball
        float trailRadiusMax = max(sphereRadius * 1.4, 0.08); // max radius near the ball
        float trailRadiusMin = 0.06;             // thin tip radius

        // find the point on the ray closest to the sphere center
        float tClosest = dot(sphereCenter - ro, rd);
        if (tClosest > 0.0 && tClosest < tMin) { // only consider trail if it's before the first hit (not occluded)
            vec3 pClosest = ro + tClosest * rd;
            // compute longitudinal coordinate along the trail (0 at just behind sphere)
            float along = dot(pClosest - (sphereCenter - velDir * trailStartOffset), -velDir);
            if (along > 0.0 && along < trailLen) {
                // center point on trail for this 'along'
                vec3 trailCenterPoint = sphereCenter - velDir * (along + trailStartOffset);
                float radial = length(pClosest - trailCenterPoint);
                float radiusAt = mix(trailRadiusMax, trailRadiusMin, smoothstep(0.0, trailLen, along));
                // smooth radial falloff and longitudinal fade
                float radialFactor = 1.0 - smoothstep(0.0, radiusAt, radial);
                float longFactor = 1.0 - (along / trailLen);
                float alpha = radialFactor * longFactor * 0.85; // overall strength
                alpha = clamp(alpha, 0.0, 1.0);
                // apply a soft blend with white (reduced strength)
                finalColor = mix(finalColor, vec3(1.0), alpha * 0.5);
            }
        }
    }

    // --- CONTRAIL HISTORY RENDERING (disks at sampled trail positions) ---
    if (nTrail > 0) {
        // iterate recent samples (newest last)
        for (int i = 0; i < nTrail; ++i) {
            int idx = nTrail - 1 - i; // start from newest
            vec3 tp = trailPositions[idx];
            // project to camera ray closest point check
            float tProj = dot(tp - ro, rd);
            if (tProj > 0.0 && tProj < tMin) {
                vec3 pOnRay = ro + tProj * rd;
                // skip sample if vertical separation is large (avoid distant artifacts)
                float ySep = abs(pOnRay.y - tp.y);
                float ageFactor = float(i) / float(nTrail); // 0 newest -> 1 oldest
                float diskRadius = mix(0.06, 0.02, ageFactor); // older samples smaller (increased for visibility)
                if (ySep > diskRadius * 3.5) continue; // relax vertical filter to keep more samples
                // use full 3D distance for a robust test
                float d3 = length(pOnRay - tp);
                float alpha = 1.0 - smoothstep(0.0, diskRadius, d3);
                alpha *= mix(0.9, 0.05, ageFactor); // fade by age
                alpha = clamp(alpha, 0.0, 1.0);
                // stronger blend for visibility
                finalColor = mix(finalColor, vec3(1.0), alpha * 0.9);
            }
        }
    }

    // clamp to avoid extreme brightness
    finalColor = clamp(finalColor, vec3(0.0), vec3(1.0));
    FragColor = vec4(finalColor, 1.0);
}