/**
 * SUPARNA — Mission Control Application v2
 * Full 3D Viewer + SPA Router + Mission Planner + Telemetry + Exports
 */

// ============================================================
//  GLOBALS
// ============================================================
var scene, camera, renderer, controls;
var missionData = null;
var droneMesh, trailLine, trailPoints = [];
var loiterMeshes = [], obstacleMeshes = [], coverageMeshes = [], labelSprites = [];
var groundPlane, gridHelper, terrainMesh;
var descentMeshes = [], allSceneObjects = [];

var simTime = 0, speed = 2.0, paused = false;
var wpIdx = 0, state = 'IDLE';
var pos = { x: 0, y: 0 }, heading = 0;
var loiterCenter = null, loiterAngle = 0, loiterRevs = 0, loiterR = 60;
var nLoitersDone = 0, distance = 0, battery = 100;
var coveredSet = new Set();
var safeWaypoints = [];
var isLAC = false;
var SPD = 35, TURN_RATE = 2.5;
var DRONE_ALT = 15;
var energyUsedWh = 0, energyCapacityWh = 370;
var descentWpIdx = 0, descentWps = [];
var currentAltAGL = 0;
var heightmapData = null, hmW = 0, hmH = 0, mapW = 1000, mapH = 700;
var hmMinElev = 0, hmMaxElev = 1;
var viewerInitialized = false;
var lastTime = 0;

// ============================================================
//  NAVIGATION
// ============================================================
function navigate(page) {
    document.querySelectorAll('.page').forEach(function (p) { p.classList.remove('active'); });
    document.querySelectorAll('.nav-item').forEach(function (n) { n.classList.remove('active'); });
    var el = document.getElementById('page-' + page);
    if (el) el.classList.add('active');
    var nav = document.querySelector('.nav-item[data-page="' + page + '"]');
    if (nav) nav.classList.add('active');

    if (page === 'viewer' && !viewerInitialized) initViewer();
    if (page === 'viewer' && renderer) onResize();
    if (page === 'telemetry') updateTelemetryPage();
    if (page === 'exports') updateExportsPage();
}

// ============================================================
//  CLOCK
// ============================================================
function updateClock() {
    var d = new Date();
    var h = String(d.getHours()).padStart(2, '0');
    var m = String(d.getMinutes()).padStart(2, '0');
    var s = String(d.getSeconds()).padStart(2, '0');
    var el = document.getElementById('clock');
    if (el) el.textContent = h + ':' + m + ':' + s;
}
setInterval(updateClock, 1000);
updateClock();

// ============================================================
//  THREE.JS VIEWER
// ============================================================
function onResize() {
    var container = document.getElementById('viewer-canvas-wrap');
    if (!container || !renderer) return;
    var w = container.clientWidth, h = container.clientHeight;
    if (w < 10 || h < 10) return;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
}

function initViewer() {
    var container = document.getElementById('viewer-canvas-wrap');
    if (!container) return;

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x050810);
    scene.fog = new THREE.FogExp2(0x050810, 0.00015);

    var w = container.clientWidth || window.innerWidth - 72;
    var h = container.clientHeight || window.innerHeight - 56;

    camera = new THREE.PerspectiveCamera(55, w / h, 1, 50000);
    camera.position.set(600, 500, 600);

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(w, h);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.0;
    container.appendChild(renderer.domElement);

    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.06;
    controls.minDistance = 50;
    controls.maxDistance = 12000;
    controls.maxPolarAngle = Math.PI * 0.48;

    // Lighting — moody cinematic
    var amb = new THREE.AmbientLight(0x1a2a44, 0.4);
    scene.add(amb);
    var sun = new THREE.DirectionalLight(0xffeedd, 0.7);
    sun.position.set(300, 600, 400);
    sun.castShadow = true;
    scene.add(sun);
    var hemi = new THREE.HemisphereLight(0x6688bb, 0x112233, 0.4);
    scene.add(hemi);
    // Soft cyan rim from camera side
    var rim = new THREE.PointLight(0x00d4ff, 0.2, 5000);
    rim.position.set(-500, 200, -500);
    scene.add(rim);

    window.addEventListener('resize', onResize);
    viewerInitialized = true;

    loadMission();
}

// ============================================================
//  LOAD MISSION
// ============================================================
async function loadMission() {
    try {
        var resp = await fetch('/mission.json');
        if (!resp.ok) throw new Error('No mission');
        missionData = await resp.json();
    } catch (e) {
        setHUD('status', 'NO MISSION');
        setHUD('phase', 'Use Mission Planner to generate');
        return;
    }

    pos = { x: missionData.home.x, y: missionData.home.y };
    isLAC = missionData.map && missionData.map.type === 'lac';

    // Energy
    if (missionData.energy) {
        energyCapacityWh = missionData.energy.battery_capacity_wh || 370;
    }
    // Descent
    if (missionData.descent && missionData.descent.waypoints) {
        descentWps = missionData.descent.waypoints;
    }
    // Heightmap
    if (missionData.heightmap && missionData.heightmap.data) {
        heightmapData = missionData.heightmap.data;
        hmW = missionData.heightmap.cols || 1;
        hmH = missionData.heightmap.rows || 1;
        hmMinElev = missionData.heightmap.min_elevation || 0;
        hmMaxElev = missionData.heightmap.max_elevation || 1;
    }
    mapW = missionData.map ? missionData.map.width : 1000;
    mapH = missionData.map ? missionData.map.height : 700;

    computeSafePath();
    buildScene();
    state = 'FLY';
    currentAltAGL = DRONE_ALT;
    energyUsedWh = 0;
    battery = 100;
    animate();
}

function setHUD(id, val) {
    var el = document.getElementById('hud-' + id);
    if (el) el.textContent = val;
}

// ============================================================
//  TERRAIN HELPERS
// ============================================================
function getTerrainHeight(x, y) {
    if (!heightmapData || !hmW || !hmH) return 0;
    var px = Math.floor((x / mapW) * (hmW - 1));
    var py = Math.floor((y / mapH) * (hmH - 1));
    px = Math.max(0, Math.min(px, hmW - 1));
    py = Math.max(0, Math.min(py, hmH - 1));
    var idx = py * hmW + px;
    var raw = heightmapData[idx] || 0;
    return ((raw - hmMinElev) / (hmMaxElev - hmMinElev || 1)) * 250;
}

function terrainColor(t) {
    t = Math.max(0, Math.min(1, t));
    if (t < 0.10) return new THREE.Color(0.08, 0.14, 0.06);
    if (t < 0.25) return new THREE.Color(0.14, 0.22, 0.10);
    if (t < 0.40) return new THREE.Color(0.22, 0.28, 0.14);
    if (t < 0.55) return new THREE.Color(0.35, 0.30, 0.18);
    if (t < 0.70) return new THREE.Color(0.48, 0.40, 0.26);
    if (t < 0.82) return new THREE.Color(0.62, 0.55, 0.40);
    if (t < 0.92) return new THREE.Color(0.78, 0.72, 0.60);
    return new THREE.Color(0.92, 0.92, 0.95);  // Snow caps
}

function buildTerrain() {
    if (!heightmapData || !hmW || !hmH) return;
    // High-res terrain mesh
    var geo = new THREE.PlaneGeometry(mapW, mapH, hmW - 1, hmH - 1);
    geo.rotateX(-Math.PI / 2);
    var verts = geo.attributes.position.array;
    var colors = new Float32Array(verts.length);

    for (var i = 0; i < hmW * hmH; i++) {
        var vi = i * 3;
        var raw = heightmapData[i] || 0;
        var norm = (raw - hmMinElev) / (hmMaxElev - hmMinElev || 1);
        verts[vi + 1] = norm * 250;
        var col = terrainColor(norm);
        colors[vi] = col.r; colors[vi + 1] = col.g; colors[vi + 2] = col.b;
    }
    geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    geo.computeVertexNormals();

    var mat = new THREE.MeshStandardMaterial({
        vertexColors: true,
        roughness: 0.85,
        metalness: 0.05,
        flatShading: false,
    });
    terrainMesh = new THREE.Mesh(geo, mat);
    terrainMesh.position.set(mapW / 2, 0, mapH / 2);
    terrainMesh.receiveShadow = true;
    scene.add(terrainMesh);
}

// ============================================================
//  TEXT SPRITE
// ============================================================
function makeTextSprite(text, color, size) {
    var canvas = document.createElement('canvas');
    var ctx = canvas.getContext('2d');
    canvas.width = 512; canvas.height = 64;
    ctx.font = 'bold ' + (size || 24) + 'px Inter, Arial, sans-serif';
    ctx.fillStyle = color || '#ffffff';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, 256, 32);
    var tex = new THREE.CanvasTexture(canvas);
    tex.minFilter = THREE.LinearFilter;
    var mat = new THREE.SpriteMaterial({ map: tex, transparent: true, depthTest: false });
    var sprite = new THREE.Sprite(mat);
    sprite.scale.set(120, 15, 1);
    return sprite;
}

// ============================================================
//  BUILD SCENE
// ============================================================
function buildScene() {
    if (!missionData || !scene) return;

    // Clear everything
    allSceneObjects.forEach(function (m) {
        scene.remove(m);
        if (m.geometry) m.geometry.dispose();
        if (m.material) { if (m.material.map) m.material.map.dispose(); m.material.dispose(); }
    });
    allSceneObjects = [];
    loiterMeshes = []; obstacleMeshes = []; coverageMeshes = []; labelSprites = []; descentMeshes = [];
    if (terrainMesh) { scene.remove(terrainMesh); terrainMesh = null; }
    if (groundPlane) { scene.remove(groundPlane); groundPlane = null; }
    if (gridHelper) { scene.remove(gridHelper); gridHelper = null; }
    if (droneMesh) { scene.remove(droneMesh); droneMesh = null; }
    if (trailLine) { scene.remove(trailLine); trailLine = null; }
    trailPoints = [];

    // === GROUND ===
    if (isLAC && heightmapData) {
        buildTerrain();
    } else {
        // Flat map with grid
        var gGeo = new THREE.PlaneGeometry(mapW + 400, mapH + 400);
        gGeo.rotateX(-Math.PI / 2);
        groundPlane = new THREE.Mesh(gGeo, new THREE.MeshStandardMaterial({
            color: 0x0a0f1a, roughness: 0.9, metalness: 0.0
        }));
        groundPlane.position.set(mapW / 2, -0.5, mapH / 2);
        groundPlane.receiveShadow = true;
        scene.add(groundPlane);

        gridHelper = new THREE.GridHelper(Math.max(mapW, mapH) + 400, 50, 0x112233, 0x0d1520);
        gridHelper.position.set(mapW / 2, 0, mapH / 2);
        scene.add(gridHelper);

        // Map boundary
        var borderGeo = new THREE.EdgesGeometry(new THREE.BoxGeometry(mapW, 2, mapH));
        var borderLine = new THREE.LineSegments(borderGeo, new THREE.LineBasicMaterial({ color: 0x1a2a40 }));
        borderLine.position.set(mapW / 2, 1, mapH / 2);
        scene.add(borderLine); allSceneObjects.push(borderLine);
    }

    // === OBSTACLES ===
    (missionData.obstacles || []).forEach(function (obs) {
        var bH = isLAC ? getTerrainHeight(obs.x, obs.y) : 0;
        if (obs.is_no_fly) {
            var r = obs.radius || 80;
            // Red no-fly zone ring
            var ring = new THREE.Mesh(
                new THREE.RingGeometry(r - 3, r + 3, 48),
                new THREE.MeshBasicMaterial({ color: 0xff2040, side: THREE.DoubleSide, transparent: true, opacity: 0.35 })
            );
            ring.rotation.x = -Math.PI / 2;
            ring.position.set(obs.x, bH + 3, obs.y);
            scene.add(ring); allSceneObjects.push(ring); obstacleMeshes.push(ring);
            // Fill
            var fill = new THREE.Mesh(
                new THREE.CircleGeometry(r, 48),
                new THREE.MeshBasicMaterial({ color: 0xff0020, transparent: true, opacity: 0.08, side: THREE.DoubleSide })
            );
            fill.rotation.x = -Math.PI / 2;
            fill.position.set(obs.x, bH + 2, obs.y);
            scene.add(fill); allSceneObjects.push(fill);
            var lbl = makeTextSprite(obs.name || 'NO-FLY', '#ff4060');
            lbl.position.set(obs.x, bH + 40, obs.y);
            scene.add(lbl); allSceneObjects.push(lbl); labelSprites.push(lbl);
        } else {
            // Mountain / obstacle cone
            var r2 = obs.radius || 40;
            var h = r2 * 1.5;
            var geo = new THREE.ConeGeometry(r2, h, 8);
            var mtl = new THREE.MeshStandardMaterial({ color: 0x3a3a4a, roughness: 0.7, transparent: true, opacity: 0.6 });
            var mesh = new THREE.Mesh(geo, mtl);
            mesh.position.set(obs.x, bH + h / 2, obs.y);
            mesh.castShadow = true;
            scene.add(mesh); allSceneObjects.push(mesh); obstacleMeshes.push(mesh);
            if (obs.name) {
                var lbl2 = makeTextSprite(obs.name, '#7788aa');
                lbl2.position.set(obs.x, bH + h + 25, obs.y);
                scene.add(lbl2); allSceneObjects.push(lbl2); labelSprites.push(lbl2);
            }
        }
    });

    // === LANDMARKS ===
    (missionData.landmarks || []).forEach(function (lm) {
        var bH = isLAC ? getTerrainHeight(lm.x, lm.y) : 0;
        var spr = makeTextSprite(lm.name, '#00d4ff', 20);
        spr.position.set(lm.x, bH + 50, lm.y);
        scene.add(spr); allSceneObjects.push(spr); labelSprites.push(spr);
        // Small pin
        var pin = new THREE.Mesh(
            new THREE.CylinderGeometry(0, 4, 20, 4),
            new THREE.MeshBasicMaterial({ color: 0x00b4d8 })
        );
        pin.position.set(lm.x, bH + 30, lm.y);
        scene.add(pin); allSceneObjects.push(pin);
    });

    // === LOITER ZONES ===
    (missionData.loiters || []).forEach(function (l, i) {
        var r = l.radius || 60;
        var bH = isLAC ? getTerrainHeight(l.x, l.y) : 0;
        // Ring
        var ring = new THREE.Mesh(
            new THREE.RingGeometry(r - 2, r + 1, 64),
            new THREE.MeshBasicMaterial({ color: 0xffb040, side: THREE.DoubleSide, transparent: true, opacity: 0.4 })
        );
        ring.rotation.x = -Math.PI / 2;
        ring.position.set(l.x, bH + DRONE_ALT, l.y);
        scene.add(ring); allSceneObjects.push(ring); loiterMeshes.push(ring);
        // Center dot
        var dot = new THREE.Mesh(
            new THREE.SphereGeometry(3.5, 10, 10),
            new THREE.MeshBasicMaterial({ color: 0xffb040 })
        );
        dot.position.set(l.x, bH + DRONE_ALT, l.y);
        scene.add(dot); allSceneObjects.push(dot); loiterMeshes.push(dot);
        // Index label
        var idx = makeTextSprite('L' + (i + 1), '#ffcc66', 18);
        idx.position.set(l.x, bH + DRONE_ALT + 18, l.y);
        scene.add(idx); allSceneObjects.push(idx); labelSprites.push(idx);
    });

    // === HOME BASE ===
    var hbH = isLAC ? getTerrainHeight(missionData.home.x, missionData.home.y) : 0;
    // Platform
    var homePlat = new THREE.Mesh(
        new THREE.CylinderGeometry(12, 14, 3, 6),
        new THREE.MeshStandardMaterial({ color: 0x10b981, emissive: 0x044d32, roughness: 0.4 })
    );
    homePlat.position.set(missionData.home.x, hbH + 1.5, missionData.home.y);
    scene.add(homePlat); allSceneObjects.push(homePlat);
    // Beacon
    var beacon = new THREE.Mesh(
        new THREE.CylinderGeometry(1, 1, 25, 6),
        new THREE.MeshBasicMaterial({ color: 0x10b981, transparent: true, opacity: 0.5 })
    );
    beacon.position.set(missionData.home.x, hbH + 14, missionData.home.y);
    scene.add(beacon); allSceneObjects.push(beacon);
    var homeLbl = makeTextSprite('HOME BASE', '#10b981');
    homeLbl.position.set(missionData.home.x, hbH + 35, missionData.home.y);
    scene.add(homeLbl); allSceneObjects.push(homeLbl); labelSprites.push(homeLbl);

    // === DRONE MODEL ===
    var droneGrp = new THREE.Group();
    // Body
    var body = new THREE.Mesh(
        new THREE.ConeGeometry(5, 20, 4),
        new THREE.MeshStandardMaterial({ color: 0x00d4ff, emissive: 0x003344, roughness: 0.3, metalness: 0.6 })
    );
    body.rotation.x = Math.PI / 2;
    droneGrp.add(body);
    // Wings
    var wingGeo = new THREE.BoxGeometry(22, 1.2, 6);
    var wingMat = new THREE.MeshStandardMaterial({ color: 0x0090b0, roughness: 0.4, metalness: 0.5 });
    var wingL = new THREE.Mesh(wingGeo, wingMat);
    wingL.position.set(-11, 0, -1);
    wingL.rotation.z = 0.05;
    droneGrp.add(wingL);
    var wingR = new THREE.Mesh(wingGeo, wingMat);
    wingR.position.set(11, 0, -1);
    wingR.rotation.z = -0.05;
    droneGrp.add(wingR);
    // Nav lights
    var navL = new THREE.Mesh(new THREE.SphereGeometry(1, 6, 6), new THREE.MeshBasicMaterial({ color: 0xff0040 }));
    navL.position.set(-20, 0, -1);
    droneGrp.add(navL);
    var navR = new THREE.Mesh(new THREE.SphereGeometry(1, 6, 6), new THREE.MeshBasicMaterial({ color: 0x00ff40 }));
    navR.position.set(20, 0, -1);
    droneGrp.add(navR);

    droneMesh = droneGrp;
    droneMesh.position.set(pos.x, hbH + DRONE_ALT, pos.y);
    droneMesh.castShadow = true;
    scene.add(droneMesh);

    // === TRAIL ===
    trailLine = new THREE.Line(
        new THREE.BufferGeometry(),
        new THREE.LineBasicMaterial({ color: 0x00d4ff, transparent: true, opacity: 0.4 })
    );
    scene.add(trailLine);

    // === DESCENT PREVIEW ===
    buildDescentPreview();

    // === CAMERA ===
    if (isLAC) {
        camera.position.set(mapW * 0.5 + 1200, 600, mapH * 0.5 + 1200);
        controls.target.set(mapW * 0.5, 80, mapH * 0.5);
    } else {
        camera.position.set(mapW * 0.5 + 400, 350, mapH * 0.5 + 400);
        controls.target.set(mapW * 0.5, 0, mapH * 0.5);
    }
    controls.update();
}

// ============================================================
//  DESCENT PREVIEW
// ============================================================
function buildDescentPreview() {
    if (!missionData || !missionData.descent || !missionData.descent.waypoints || missionData.descent.waypoints.length < 2) return;
    var wps = missionData.descent.waypoints;
    var pts = [];
    // Sample every 5th waypoint for performance
    for (var i = 0; i < wps.length; i += 5) {
        var w = wps[i];
        var bh = isLAC ? getTerrainHeight(w.x, w.y) : 0;
        pts.push(new THREE.Vector3(w.x, bh + (w.alt || 0), w.y));
    }
    if (pts.length < 2) return;

    var line = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(pts),
        new THREE.LineBasicMaterial({ color: 0xffcc00, transparent: true, opacity: 0.3, linewidth: 2 })
    );
    scene.add(line); allSceneObjects.push(line); descentMeshes.push(line);

    // Landing zone ring
    if (missionData.descent.center) {
        var cx = missionData.descent.center.x, cy = missionData.descent.center.y;
        var lr = missionData.descent.radius_m || 60;
        var lh = isLAC ? getTerrainHeight(cx, cy) : 0;
        var ring = new THREE.Mesh(
            new THREE.RingGeometry(lr * 0.3, lr * 0.38, 48),
            new THREE.MeshBasicMaterial({ color: 0xffcc00, side: THREE.DoubleSide, transparent: true, opacity: 0.2 })
        );
        ring.rotation.x = -Math.PI / 2;
        ring.position.set(cx, lh + 1, cy);
        scene.add(ring); allSceneObjects.push(ring); descentMeshes.push(ring);

        var lbl = makeTextSprite('LANDING ZONE', '#ffcc00');
        lbl.position.set(cx, lh + 35, cy);
        scene.add(lbl); allSceneObjects.push(lbl); descentMeshes.push(lbl);
    }
}

// ============================================================
//  SAFE PATH
// ============================================================
function normalizeAngle(a) {
    while (a > Math.PI) a -= 2 * Math.PI;
    while (a < -Math.PI) a += 2 * Math.PI;
    return a;
}

function computeSafePath() {
    if (!missionData) return;
    safeWaypoints = [];
    (missionData.loiters || []).forEach(function (l) {
        safeWaypoints.push({ x: l.x, y: l.y, type: 'loiter', radius: l.radius || 60 });
    });
    safeWaypoints.push({ x: missionData.home.x, y: missionData.home.y, type: 'return' });
}

// ============================================================
//  COVERAGE
// ============================================================
function markCoverage(cx, cy, rad) {
    if (!scene) return;
    var res = (missionData && missionData.map) ? (missionData.map.resolution || 10) : 10;
    var r2 = rad * rad;
    var step = isLAC ? res * 2 : res;
    for (var dx = -rad; dx <= rad; dx += step) {
        for (var dy = -rad; dy <= rad; dy += step) {
            if (dx * dx + dy * dy > r2) continue;
            var gx = Math.floor((cx + dx) / step), gy = Math.floor((cy + dy) / step);
            var key = gx + ',' + gy;
            if (!coveredSet.has(key)) {
                coveredSet.add(key);
                var px = gx * step, py = gy * step;
                var bH = isLAC ? getTerrainHeight(px, py) : 0;
                var sz = step * 0.85;
                var covMesh = new THREE.Mesh(
                    new THREE.PlaneGeometry(sz, sz),
                    new THREE.MeshBasicMaterial({ color: 0x10b981, transparent: true, opacity: 0.15, side: THREE.DoubleSide })
                );
                covMesh.rotation.x = -Math.PI / 2;
                covMesh.position.set(px, bH + 0.8, py);
                scene.add(covMesh); coverageMeshes.push(covMesh);
            }
        }
    }
}

// ============================================================
//  SIMULATION LOOP
// ============================================================
function updateSim(dt) {
    if (paused || state === 'DONE' || state === 'IDLE' || !missionData) return;

    var powerW = missionData.performance ? missionData.performance.power_draw_w : 133;
    var cruiseSpd = missionData.performance ? missionData.performance.cruise_speed_ms : 19;

    // === DESCENT ===
    if (state === 'DESCENT') {
        if (descentWpIdx >= descentWps.length) {
            state = 'DONE'; currentAltAGL = 0; updateDroneMesh(); return;
        }
        var dw = descentWps[descentWpIdx];
        var dx = dw.x - pos.x, dy = dw.y - pos.y;
        var dist = Math.sqrt(dx * dx + dy * dy);
        if (dist < 12) {
            descentWpIdx++;
        } else {
            var th = Math.atan2(dy, dx);
            var spd = (dw.speed || cruiseSpd) * 0.5;
            pos.x += spd * dt * Math.cos(th);
            pos.y += spd * dt * Math.sin(th);
            heading = th;
        }
        currentAltAGL = dw.alt || 0;
        distance += SPD * dt * 0.3;
        energyUsedWh += powerW * 0.5 * dt / 3600;
        battery = Math.max(0, (1 - energyUsedWh / energyCapacityWh) * 100);
        updateDroneMesh();
        return;
    }

    // === LOITER ===
    if (state === 'LOITER') {
        if (!loiterCenter) { state = 'FLY'; return; }
        var w = SPD / loiterR;
        loiterAngle += w * dt;
        loiterRevs += w * dt / (2 * Math.PI);
        pos.x = loiterCenter.x + loiterR * Math.cos(loiterAngle);
        pos.y = loiterCenter.y + loiterR * Math.sin(loiterAngle);
        heading = normalizeAngle(loiterAngle + Math.PI / 2);
        distance += SPD * dt;
        energyUsedWh += powerW * 0.92 * dt / 3600;
        battery = Math.max(0, (1 - energyUsedWh / energyCapacityWh) * 100);
        currentAltAGL = DRONE_ALT;
        markCoverage(loiterCenter.x, loiterCenter.y, loiterR + 30);
        if (loiterRevs >= 1.0) {
            nLoitersDone++; state = 'FLY'; loiterCenter = null; wpIdx++;
        }
        updateDroneMesh();
        return;
    }

    // === FLY ===
    if (wpIdx >= safeWaypoints.length) {
        if (descentWps.length > 0) {
            state = 'DESCENT'; descentWpIdx = 0; currentAltAGL = DRONE_ALT;
        } else {
            state = 'DONE';
        }
        return;
    }

    var target = safeWaypoints[wpIdx];
    var distT = Math.sqrt((target.x - pos.x) ** 2 + (target.y - pos.y) ** 2);
    var th2 = Math.atan2(target.y - pos.y, target.x - pos.x);
    var err = normalizeAngle(th2 - heading);
    var mt = TURN_RATE * dt;
    if (Math.abs(err) > mt) heading += err > 0 ? mt : -mt;
    else heading = th2;
    heading = normalizeAngle(heading);
    var d = SPD * dt;
    pos.x += d * Math.cos(heading);
    pos.y += d * Math.sin(heading);
    distance += d;
    energyUsedWh += powerW * dt / 3600;
    battery = Math.max(0, (1 - energyUsedWh / energyCapacityWh) * 100);
    currentAltAGL = DRONE_ALT;
    markCoverage(pos.x, pos.y, isLAC ? 140 : 50);

    var captureR = (target.type === 'return') ? 80 : (isLAC ? 60 : 25);
    if (distT < captureR) {
        if (target.type === 'loiter') {
            state = 'LOITER';
            loiterCenter = { x: target.x, y: target.y };
            loiterAngle = heading - Math.PI / 2;
            loiterRevs = 0;
            loiterR = target.radius || 60;
        } else {
            wpIdx++;
        }
    }
    updateDroneMesh();
}

// ============================================================
//  DRONE MESH UPDATE
// ============================================================
function updateDroneMesh() {
    if (!droneMesh) return;
    var baseH = isLAC ? getTerrainHeight(pos.x, pos.y) : 0;
    var alt = (state === 'DESCENT') ? currentAltAGL : DRONE_ALT;
    var bob = (state === 'DESCENT') ? Math.sin(simTime * 4) * 0.3 : Math.sin(simTime * 2.5) * 2;
    droneMesh.position.set(pos.x, baseH + alt + bob, pos.y);
    droneMesh.rotation.y = -heading + Math.PI / 2;
    // Bank in turns
    if (state === 'LOITER') {
        droneMesh.rotation.z = 0.45;
    } else if (state === 'DESCENT') {
        droneMesh.rotation.z = Math.sin(simTime * 1.5) * 0.35;
    } else {
        droneMesh.rotation.z *= 0.9; // Smooth unbank
    }

    var trailH = baseH + alt - 2;
    trailPoints.push(new THREE.Vector3(pos.x, trailH, pos.y));
    if (trailPoints.length > 5000) trailPoints.shift();
    if (trailLine && trailPoints.length > 1) {
        trailLine.geometry.dispose();
        trailLine.geometry = new THREE.BufferGeometry().setFromPoints(trailPoints);
    }
}

// ============================================================
//  HUD UPDATE
// ============================================================
function updateHUD() {
    if (!missionData) return;
    var totalLoiters = missionData.loiters ? missionData.loiters.length : 0;
    var res = (missionData.map && missionData.map.resolution) || 10;
    var step = isLAC ? res * 2 : res;
    var totalCells = Math.floor(mapW / step) * Math.floor(mapH / step);
    var covPct = totalCells > 0 ? Math.min(100, (coveredSet.size / totalCells) * 100) : 0;

    // Top-left HUD
    var statusText = state === 'DONE' ? 'MISSION COMPLETE' :
        state === 'DESCENT' ? 'LOITER-TO-LAND' :
            state === 'LOITER' ? 'LOITERING' :
                state === 'FLY' ? 'EN ROUTE' : 'STANDBY';
    setHUD('status', statusText);

    var phaseText = state === 'DONE' ? 'All loiters complete. Touchdown.' :
        state === 'DESCENT' ? 'Spiral descent: ' + currentAltAGL.toFixed(0) + 'm AGL' :
            state === 'LOITER' ? 'Surveillance orbit L' + (nLoitersDone + 1) + '/' + totalLoiters :
                state === 'FLY' ? 'Transit to L' + (nLoitersDone + 1) : 'Awaiting mission data';
    setHUD('phase', phaseText);

    setHUD('loiters', nLoitersDone + '/' + totalLoiters);
    setHUD('distance', distance >= 1000 ? (distance / 1000).toFixed(1) + ' km' : distance.toFixed(0) + ' m');
    setHUD('coverage', covPct.toFixed(1) + '%');

    // Bottom strip
    var cruiseSpd = missionData.performance ? missionData.performance.cruise_speed_ms : 18;
    setHUD('alt', (state === 'DESCENT' ? currentAltAGL.toFixed(0) : DRONE_ALT) + 'm');
    setHUD('spd', (state === 'DESCENT' ? (cruiseSpd * 0.5).toFixed(1) : cruiseSpd) + ' m/s');
    setHUD('hdg', ((heading * 180 / Math.PI + 360) % 360).toFixed(0) + '\u00B0');
    var remainWh = Math.max(0, energyCapacityWh - energyUsedWh);
    setHUD('energy', remainWh.toFixed(0) + ' Wh');

    var batEl = document.getElementById('hud-bat');
    if (batEl) {
        batEl.textContent = battery.toFixed(0) + '%';
        batEl.className = 'hud-stat-value ' + (battery < 20 ? '' : battery < 40 ? 'amber' : 'green');
    }

    var stateEl = document.getElementById('hud-state');
    if (stateEl) {
        stateEl.textContent = state;
        stateEl.className = 'hud-stat-value ' + (state === 'DONE' ? 'green' : state === 'DESCENT' ? 'amber' : 'cyan');
    }

    var speedEl = document.getElementById('hud-speed');
    if (speedEl) speedEl.textContent = speed.toFixed(1) + 'x';

    // Loiter mesh color updates
    (missionData.loiters || []).forEach(function (l, i) {
        var mesh = loiterMeshes[i * 2];
        if (!mesh) return;
        if (i < nLoitersDone) { mesh.material.color.set(0x10b981); mesh.material.opacity = 0.6; }
        else if (i === nLoitersDone && state === 'LOITER') { mesh.material.color.set(0x00d4ff); mesh.material.opacity = 0.8; }
    });
}

// ============================================================
//  ANIMATION
// ============================================================
function animate(now) {
    requestAnimationFrame(animate);
    if (!now) now = performance.now();
    var realDt = (now - (lastTime || now)) / 1000;
    lastTime = now;
    if (realDt > 0.1) realDt = 0.016; // Cap
    var dt = speed * realDt;
    simTime += dt;
    updateSim(dt);
    updateHUD();
    if (controls) controls.update();
    if (renderer && scene && camera) renderer.render(scene, camera);
}

// ============================================================
//  VIEWER CONTROLS
// ============================================================
function viewerTogglePause() {
    paused = !paused;
    var btn = document.getElementById('btnPlayPause');
    if (btn) btn.textContent = paused ? 'Play' : 'Pause';
}

function viewerSlower() { speed = Math.max(0.5, speed - 0.5); }
function viewerFaster() { speed = Math.min(10, speed + 0.5); }

function viewerReset() {
    if (!missionData) return;
    wpIdx = 0; state = 'FLY'; nLoitersDone = 0; distance = 0; battery = 100;
    energyUsedWh = 0; descentWpIdx = 0; currentAltAGL = DRONE_ALT;
    pos = { x: missionData.home.x, y: missionData.home.y };
    heading = 0; trailPoints = []; coveredSet.clear(); paused = false;
    var btn = document.getElementById('btnPlayPause');
    if (btn) btn.textContent = 'Pause';
    coverageMeshes.forEach(function (m) { scene.remove(m); if (m.geometry) m.geometry.dispose(); if (m.material) m.material.dispose(); });
    coverageMeshes = [];
    loiterMeshes.forEach(function (m, i) {
        if (i % 2 === 0 && m.material) { m.material.color.set(0xffb040); m.material.opacity = 0.4; }
    });
    computeSafePath();
}

// ============================================================
//  MISSION PLANNER
// ============================================================
function initPlanner() {
    var slider = document.getElementById('plannerAlt');
    if (slider) {
        slider.addEventListener('input', function () {
            document.getElementById('plannerAltVal').textContent = this.value + 'm';
            updateISAPreview(parseFloat(this.value));
        });
        updateISAPreview(4000);
    }
}

async function updateISAPreview(alt) {
    var el = document.getElementById('isaData');
    if (!el) return;
    try {
        var resp = await fetch('/api/performance/' + alt);
        if (!resp.ok) throw new Error();
        var d = await resp.json();
        el.innerHTML =
            '<b>Cruise:</b> ' + d.cruise_speed_ms + ' m/s<br>' +
            '<b>Power:</b> ' + d.power_draw_w + ' W<br>' +
            '<b>Loiter R:</b> ' + d.loiter_radius_m + ' m<br>' +
            '<b>Density:</b> ' + d.air_density + ' kg/m\u00B3 (\u03C3=' + d.density_ratio + ')<br>' +
            '<b>Temp:</b> ' + d.temperature_c + '\u00B0C<br>' +
            '<b>Endurance:</b> ' + (d.endurance ? d.endurance.endurance_hours : '--') + ' hr';
    } catch (e) {
        el.innerHTML = '<span style="color:var(--text-muted)">Start server for live ISA data</span>';
    }
}

async function generateMission() {
    var btn = document.getElementById('btnGenerate');
    btn.textContent = 'Generating...'; btn.disabled = true;

    var mapType = document.getElementById('plannerMap').value;
    var alt = parseFloat(document.getElementById('plannerAlt').value);
    var seedVal = document.getElementById('plannerSeed').value;

    try {
        var resp = await fetch('/api/mission/generate', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ map_type: mapType, altitude_m: alt, seed: seedVal ? parseInt(seedVal) : null })
        });
        var data = await resp.json();
        if (data.success || data.loiter_count) {
            var resDiv = document.getElementById('plannerResult');
            resDiv.style.display = 'block';
            document.getElementById('plannerStats').innerHTML =
                '<span style="color:#10b981">Mission generated successfully!</span><br>' +
                '<b>Loiters:</b> ' + (data.loiter_count || '?') + '<br>' +
                '<b>Waypoints:</b> ' + (data.waypoint_count || '?') + '<br>' +
                '<b>Energy:</b> ' + ((data.energy && data.energy.total_energy_wh) || '?') + ' Wh<br>' +
                '<b>Descent:</b> ' + ((data.descent && data.descent.n_loops) || '?') + ' loops';
            // Reload viewer
            viewerInitialized = false;
            if (renderer) {
                var container = document.getElementById('viewer-canvas-wrap');
                if (container && renderer.domElement.parentElement === container) container.removeChild(renderer.domElement);
                renderer.dispose(); renderer = null;
            }
            await initViewer();
            updateTelemetryPage();
        } else {
            alert('Error: ' + JSON.stringify(data.detail || data));
        }
    } catch (e) {
        alert('Failed: ' + e.message);
    }
    btn.textContent = 'Generate Mission'; btn.disabled = false;
}

// ============================================================
//  TELEMETRY PAGE
// ============================================================
function updateTelemetryPage() {
    if (!missionData) return;
    var perf = missionData.performance || {};
    var energy = missionData.energy || {};
    var ebt = energy.energy_by_type || {};

    // Cards
    setText('tel-speed', perf.cruise_speed_ms || '--');
    setText('tel-power', perf.power_draw_w || '--');
    setText('tel-radius', perf.loiter_radius_m || '--');
    setText('tel-density', perf.density_ratio || '--');

    // Donut
    drawEnergyDonut(ebt, energy);

    // Timeline
    drawPhaseTimeline(energy);

    // Summary
    var descent = missionData.descent || {};
    setText('missionSummary',
        '<b>Loiters:</b> ' + (missionData.loiters ? missionData.loiters.length : 0) +
        '<br><b>Total Energy:</b> ' + (energy.total_energy_wh || 0).toFixed(1) + ' / ' + (energy.battery_capacity_wh || 370) + ' Wh' +
        '<br><b>Remaining:</b> ' + (energy.remaining_pct || 0).toFixed(1) + '%' +
        '<br><b>Duration:</b> ' + (energy.total_duration_min || 0).toFixed(1) + ' min' +
        '<br><b>Transit:</b> ' + ((energy.total_distance_m || 0) / 1000).toFixed(1) + ' km' +
        '<br><b>Descent:</b> ' + (descent.n_loops || 0) + ' loops, ' + (descent.energy_wh || 0).toFixed(1) + ' Wh' +
        '<br><b>Map:</b> ' + mapW + ' x ' + mapH + 'm'
    );
}

function setText(id, html) {
    var el = document.getElementById(id);
    if (el) el.innerHTML = html;
}

function drawEnergyDonut(ebt, energy) {
    var canvas = document.getElementById('energyDonut');
    if (!canvas) return;
    var ctx = canvas.getContext('2d');
    var cx = 140, cy = 140, outerR = 120, innerR = 78;
    ctx.clearRect(0, 0, 280, 280);

    var total = energy.battery_capacity_wh || 370;
    var used = energy.total_energy_wh || 0;

    var slices = [
        { label: 'Climb', value: ebt.climb || 0, color: '#00d4ff' },
        { label: 'Transit', value: ebt.transit || 0, color: '#10b981' },
        { label: 'Loiter', value: ebt.loiter || 0, color: '#f59e0b' },
        { label: 'RTB', value: ebt.rtb || 0, color: '#a855f7' },
        { label: 'Descent', value: ebt.descent || 0, color: '#ef4444' },
        { label: 'Reserve', value: Math.max(0, total - used), color: '#1e2a3a' },
    ];

    var startAngle = -Math.PI / 2;
    slices.forEach(function (s) {
        if (s.value <= 0) return;
        var arc = (s.value / total) * 2 * Math.PI;
        ctx.beginPath();
        ctx.arc(cx, cy, outerR, startAngle, startAngle + arc);
        ctx.arc(cx, cy, innerR, startAngle + arc, startAngle, true);
        ctx.closePath();
        ctx.fillStyle = s.color;
        ctx.fill();
        startAngle += arc;
    });

    // Center text
    ctx.fillStyle = '#f1f5f9';
    ctx.font = 'bold 32px Inter, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(used.toFixed(0), cx, cy - 6);
    ctx.font = '12px Inter, sans-serif';
    ctx.fillStyle = '#64748b';
    ctx.fillText('Wh / ' + total, cx, cy + 14);

    var legend = document.getElementById('energyLegend');
    if (legend) {
        legend.innerHTML = slices.filter(function (s) { return s.value > 0.1; }).map(function (s) {
            return '<span style="display:inline-flex;align-items:center;gap:5px;margin-right:12px;margin-bottom:4px"><span style="width:10px;height:10px;border-radius:2px;background:' + s.color + ';display:inline-block"></span>' + s.label + ': ' + s.value.toFixed(1) + '</span>';
        }).join('');
    }
}

function drawPhaseTimeline(energy) {
    var el = document.getElementById('phaseTimeline');
    if (!el) return;
    var total = energy.total_duration_min || 1;
    var ebt = energy.energy_by_type || {};

    // Estimate phase times from energy proportions
    var items = [
        { label: 'Climb', pct: ((ebt.climb || 0) / (energy.total_energy_wh || 1)) * 100, color: '#00d4ff' },
        { label: 'Transit', pct: ((ebt.transit || 0) / (energy.total_energy_wh || 1)) * 100, color: '#10b981' },
        { label: 'Loiter', pct: ((ebt.loiter || 0) / (energy.total_energy_wh || 1)) * 100, color: '#f59e0b' },
        { label: 'RTB', pct: ((ebt.rtb || 0) / (energy.total_energy_wh || 1)) * 100, color: '#a855f7' },
        { label: 'Descent', pct: ((ebt.descent || 0) / (energy.total_energy_wh || 1)) * 100, color: '#ef4444' },
    ];

    var html = '<div style="display:flex;height:28px;border-radius:8px;overflow:hidden;margin-bottom:12px">';
    items.forEach(function (it) {
        if (it.pct > 0.5) {
            html += '<div style="width:' + it.pct + '%;background:' + it.color + ';display:flex;align-items:center;justify-content:center;font-size:9px;font-weight:700;color:#fff;min-width:24px" title="' + it.label + '">' + it.label + '</div>';
        }
    });
    html += '</div>';
    html += '<div style="display:flex;gap:14px;flex-wrap:wrap">';
    items.forEach(function (it) {
        var time = (it.pct / 100) * total;
        html += '<span style="font-size:11px;color:#94a3b8"><span style="display:inline-block;width:8px;height:8px;border-radius:2px;background:' + it.color + ';margin-right:4px"></span>' + it.label + ': ' + time.toFixed(1) + ' min</span>';
    });
    html += '</div>';
    el.innerHTML = html;
}

// ============================================================
//  EXPORTS PAGE
// ============================================================
async function updateExportsPage() {
    if (!missionData) return;
    var energy = missionData.energy || {};
    var perf = missionData.performance || {};
    var descent = missionData.descent || {};

    var stats = [
        ['Map Type', (missionData.map || {}).type || '--'],
        ['Map Size', mapW + ' x ' + mapH + ' m'],
        ['Altitude', (missionData.altitude_m || 0) + 'm AMSL'],
        ['Loiter Zones', (missionData.loiters || []).length],
        ['Obstacles', (missionData.obstacles || []).length],
        ['Home Base', '(' + missionData.home.x + ', ' + missionData.home.y + ')'],
        ['Cruise Speed', (perf.cruise_speed_ms || '--') + ' m/s'],
        ['Power Draw', (perf.power_draw_w || '--') + ' W'],
        ['Loiter Radius', (perf.loiter_radius_m || '--') + ' m'],
        ['Density Ratio', (perf.density_ratio || '--') + ' \u03C3'],
        ['Total Energy', (energy.total_energy_wh || 0).toFixed(1) + ' Wh'],
        ['Battery', (energy.battery_capacity_wh || 370) + ' Wh'],
        ['Remaining', (energy.remaining_pct || 0).toFixed(1) + '%'],
        ['Duration', (energy.total_duration_min || 0).toFixed(1) + ' min'],
        ['Descent Loops', descent.n_loops || 0],
        ['Descent Energy', (descent.energy_wh || 0).toFixed(1) + ' Wh'],
    ];

    var tbody = document.getElementById('statsBody');
    if (tbody) {
        tbody.innerHTML = stats.map(function (s) {
            return '<tr><td>' + s[0] + '</td><td class="font-mono">' + s[1] + '</td></tr>';
        }).join('');
    }

    // ISA table
    var isaBody = document.getElementById('isaTable');
    if (isaBody) {
        var alts = [0, 1000, 2000, 3000, 4000, 5000];
        var rows = '';
        for (var i = 0; i < alts.length; i++) {
            try {
                var resp = await fetch('/api/performance/' + alts[i]);
                if (resp.ok) {
                    var d = await resp.json();
                    var highlight = (alts[i] === (missionData.altitude_m || 0)) ? ' style="color:var(--accent);font-weight:700"' : '';
                    rows += '<tr' + highlight + '><td class="font-mono">' + alts[i] + 'm</td><td>' + d.cruise_speed_ms + '</td><td>' + d.power_draw_w + '</td><td>' + d.loiter_radius_m + '</td><td>' + d.stall_speed_ms + '</td></tr>';
                }
            } catch (e) { /* skip */ }
        }
        isaBody.innerHTML = rows || '<tr><td colspan="5" style="color:var(--text-muted)">Start FastAPI server for ISA data</td></tr>';
    }
}

// ============================================================
//  INIT
// ============================================================
document.addEventListener('DOMContentLoaded', function () {
    initPlanner();
    // Auto-init viewer since it's the default page
    setTimeout(function () {
        initViewer();
    }, 100);
});
