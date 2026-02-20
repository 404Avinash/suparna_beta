/**
 * SUPARNA — Mission Control Application
 * SPA Router + 3D Viewer + Mission Planner + Telemetry + Exports
 */

// ============================================================
//  GLOBALS
// ============================================================
var scene, camera, renderer, controls;
var missionData = null;
var droneMesh, trailLine, trailPoints = [];
var loiterMeshes = [], obstacleMeshes = [], coverageMeshes = [], labelSprites = [];
var groundPlane, gridHelper, terrainMesh;
var descentMeshes = [];

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
var heightmapData = null, hmW = 0, hmH = 0, mapW = 0, mapH = 0;
var viewerInitialized = false;

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

    // Init viewer on first visit
    if (page === 'viewer' && !viewerInitialized) {
        initViewer();
    }

    // Refresh telemetry when visiting that page
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
    document.getElementById('clock').textContent = h + ':' + m + ':' + s;
}
setInterval(updateClock, 1000);
updateClock();

// ============================================================
//  THREE.JS VIEWER
// ============================================================
function initViewer() {
    var container = document.getElementById('viewer-canvas-wrap');
    if (!container) return;

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x060a13);
    scene.fog = new THREE.FogExp2(0x060a13, 0.00018);

    camera = new THREE.PerspectiveCamera(60, container.clientWidth / container.clientHeight, 1, 50000);
    camera.position.set(500, 400, 500);

    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = true;
    container.appendChild(renderer.domElement);

    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;
    controls.minDistance = 50;
    controls.maxDistance = 8000;

    // Lighting
    var amb = new THREE.AmbientLight(0x334466, 0.5);
    scene.add(amb);
    var dir = new THREE.DirectionalLight(0xffeedd, 0.8);
    dir.position.set(200, 500, 300);
    scene.add(dir);
    var hemi = new THREE.HemisphereLight(0x88aacc, 0x222233, 0.3);
    scene.add(hemi);

    window.addEventListener('resize', function () {
        camera.aspect = container.clientWidth / container.clientHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(container.clientWidth, container.clientHeight);
    });

    viewerInitialized = true;
    loadMission();
    animate();
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
        document.getElementById('hud-status').textContent = 'NO MISSION';
        document.getElementById('hud-phase').textContent = 'Use Planner to generate';
        return;
    }

    pos = { x: missionData.home.x, y: missionData.home.y };
    isLAC = missionData.map && missionData.map.type === 'lac';

    if (missionData.energy) {
        energyCapacityWh = missionData.energy.battery_capacity_wh || 370;
    }
    if (missionData.descent && missionData.descent.waypoints) {
        descentWps = missionData.descent.waypoints;
    }
    if (missionData.heightmap && missionData.heightmap.data) {
        heightmapData = missionData.heightmap.data;
        hmW = missionData.heightmap.cols || 0;
        hmH = missionData.heightmap.rows || 0;
    }
    mapW = missionData.map ? missionData.map.width : 1000;
    mapH = missionData.map ? missionData.map.height : 1000;

    computeSafePath();
    buildScene();
    state = 'FLY';
    currentAltAGL = DRONE_ALT;
}

// ============================================================
//  TERRAIN
// ============================================================
function getTerrainHeight(x, y) {
    if (!heightmapData || !hmW || !hmH) return 0;
    var px = Math.floor((x / mapW) * (hmW - 1));
    var py = Math.floor((y / mapH) * (hmH - 1));
    px = Math.max(0, Math.min(px, hmW - 1));
    py = Math.max(0, Math.min(py, hmH - 1));
    var idx = py * hmW + px;
    var raw = heightmapData[idx] || 0;
    var minE = (missionData.heightmap && missionData.heightmap.min_elevation) || 0;
    var maxE = (missionData.heightmap && missionData.heightmap.max_elevation) || 1;
    return ((raw - minE) / (maxE - minE)) * 200;
}

function terrainColor(val) {
    var t = Math.max(0, Math.min(1, val));
    if (t < 0.15) return new THREE.Color(0.18, 0.22, 0.12);
    if (t < 0.35) return new THREE.Color(0.25, 0.32, 0.18);
    if (t < 0.55) return new THREE.Color(0.4, 0.35, 0.2);
    if (t < 0.75) return new THREE.Color(0.55, 0.45, 0.3);
    if (t < 0.9) return new THREE.Color(0.7, 0.65, 0.55);
    return new THREE.Color(0.9, 0.9, 0.92);
}

function buildTerrain() {
    if (!heightmapData || !hmW || !hmH) return;
    var geo = new THREE.PlaneGeometry(mapW, mapH, hmW - 1, hmH - 1);
    geo.rotateX(-Math.PI / 2);
    var verts = geo.attributes.position.array;
    var colors = new Float32Array(verts.length);
    var minE = (missionData.heightmap && missionData.heightmap.min_elevation) || 0;
    var maxE = (missionData.heightmap && missionData.heightmap.max_elevation) || 1;

    for (var i = 0; i < hmW * hmH; i++) {
        var r = Math.floor(i / hmW), c = i % hmW;
        var vi = (r * hmW + c) * 3;
        var raw = heightmapData[i] || 0;
        var norm = (raw - minE) / (maxE - minE);
        verts[vi + 1] = norm * 200;
        var col = terrainColor(norm);
        colors[vi] = col.r; colors[vi + 1] = col.g; colors[vi + 2] = col.b;
    }
    geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    geo.computeVertexNormals();

    var mat = new THREE.MeshLambertMaterial({ vertexColors: true });
    terrainMesh = new THREE.Mesh(geo, mat);
    terrainMesh.position.set(mapW / 2, 0, mapH / 2);
    scene.add(terrainMesh);
}

// ============================================================
//  TEXT SPRITE
// ============================================================
function makeTextSprite(text, color) {
    var canvas = document.createElement('canvas');
    var ctx = canvas.getContext('2d');
    canvas.width = 256; canvas.height = 64;
    ctx.font = 'bold 22px Inter, sans-serif';
    ctx.fillStyle = color || '#ffffff';
    ctx.textAlign = 'center';
    ctx.fillText(text, 128, 40);
    var tex = new THREE.CanvasTexture(canvas);
    var mat = new THREE.SpriteMaterial({ map: tex, transparent: true });
    var sprite = new THREE.Sprite(mat);
    sprite.scale.set(80, 20, 1);
    return sprite;
}

// ============================================================
//  BUILD SCENE
// ============================================================
function buildScene() {
    if (!missionData) return;

    // Clear
    [loiterMeshes, obstacleMeshes, coverageMeshes, labelSprites, descentMeshes].forEach(function (arr) {
        arr.forEach(function (m) { scene.remove(m); if (m.geometry) m.geometry.dispose(); if (m.material) m.material.dispose(); });
        arr.length = 0;
    });
    if (terrainMesh) { scene.remove(terrainMesh); terrainMesh = null; }
    if (groundPlane) { scene.remove(groundPlane); }
    if (gridHelper) { scene.remove(gridHelper); }
    if (droneMesh) { scene.remove(droneMesh); }
    if (trailLine) { scene.remove(trailLine); }
    trailPoints = [];

    // Ground
    if (isLAC && heightmapData) {
        buildTerrain();
    } else {
        var gGeo = new THREE.PlaneGeometry(mapW + 200, mapH + 200);
        gGeo.rotateX(-Math.PI / 2);
        groundPlane = new THREE.Mesh(gGeo, new THREE.MeshLambertMaterial({ color: 0x0d1117 }));
        groundPlane.position.set(mapW / 2, -0.5, mapH / 2);
        scene.add(groundPlane);

        gridHelper = new THREE.GridHelper(Math.max(mapW, mapH) + 200, 40, 0x1a2233, 0x111827);
        gridHelper.position.set(mapW / 2, 0, mapH / 2);
        scene.add(gridHelper);
    }

    // Obstacles
    if (missionData.obstacles) {
        missionData.obstacles.forEach(function (obs) {
            var h = obs.height || 60;
            var bH = isLAC ? getTerrainHeight(obs.x, obs.y) : 0;
            if (obs.type === 'no_fly_zone') {
                var r = obs.radius || 80;
                var ring = new THREE.Mesh(
                    new THREE.RingGeometry(r - 2, r + 2, 32),
                    new THREE.MeshBasicMaterial({ color: 0xff2040, side: THREE.DoubleSide, transparent: true, opacity: 0.4 })
                );
                ring.rotation.x = -Math.PI / 2;
                ring.position.set(obs.x, bH + 2, obs.y);
                scene.add(ring); obstacleMeshes.push(ring);
                var label = makeTextSprite(obs.name || 'NO-FLY', '#ff2040');
                label.position.set(obs.x, bH + 30, obs.y);
                scene.add(label); labelSprites.push(label);
            } else {
                var r2 = obs.radius || 40;
                var geo = new THREE.ConeGeometry(r2, h, 8);
                var mtl = new THREE.MeshLambertMaterial({ color: 0x444455, transparent: true, opacity: 0.6 });
                var mesh = new THREE.Mesh(geo, mtl);
                mesh.position.set(obs.x, bH + h / 2, obs.y);
                scene.add(mesh); obstacleMeshes.push(mesh);
                if (obs.name) {
                    var lbl = makeTextSprite(obs.name, '#8899aa');
                    lbl.position.set(obs.x, bH + h + 20, obs.y);
                    scene.add(lbl); labelSprites.push(lbl);
                }
            }
        });
    }

    // Landmarks
    if (missionData.landmarks) {
        missionData.landmarks.forEach(function (lm) {
            var bH = isLAC ? getTerrainHeight(lm.x, lm.y) : 0;
            var spr = makeTextSprite(lm.name, '#00d4ff');
            spr.position.set(lm.x, bH + 40, lm.y);
            scene.add(spr); labelSprites.push(spr);
        });
    }

    // Loiter zones
    if (missionData.loiters) {
        missionData.loiters.forEach(function (l) {
            var r = l.radius || 60;
            var bH = isLAC ? getTerrainHeight(l.x, l.y) : 0;
            var ring = new THREE.Mesh(
                new THREE.RingGeometry(r - 3, r, 32),
                new THREE.MeshBasicMaterial({ color: 0xffb040, side: THREE.DoubleSide, transparent: true, opacity: 0.5 })
            );
            ring.rotation.x = -Math.PI / 2;
            ring.position.set(l.x, bH + DRONE_ALT, l.y);
            scene.add(ring); loiterMeshes.push(ring);
            var dot = new THREE.Mesh(
                new THREE.SphereGeometry(3, 8, 8),
                new THREE.MeshBasicMaterial({ color: 0xffb040 })
            );
            dot.position.set(l.x, bH + DRONE_ALT, l.y);
            scene.add(dot); loiterMeshes.push(dot);
        });
    }

    // Home base
    var hbH = isLAC ? getTerrainHeight(missionData.home.x, missionData.home.y) : 0;
    var homeMarker = new THREE.Mesh(
        new THREE.CylinderGeometry(8, 8, 3, 6),
        new THREE.MeshBasicMaterial({ color: 0x10b981 })
    );
    homeMarker.position.set(missionData.home.x, hbH + 2, missionData.home.y);
    scene.add(homeMarker);
    var homeLbl = makeTextSprite('HOME', '#10b981');
    homeLbl.position.set(missionData.home.x, hbH + 25, missionData.home.y);
    scene.add(homeLbl); labelSprites.push(homeLbl);

    // Drone
    var droneGrp = new THREE.Group();
    var body = new THREE.Mesh(
        new THREE.ConeGeometry(5, 16, 4),
        new THREE.MeshLambertMaterial({ color: 0x00d4ff })
    );
    body.rotation.x = Math.PI / 2;
    droneGrp.add(body);
    var wingL = new THREE.Mesh(
        new THREE.BoxGeometry(18, 1, 5),
        new THREE.MeshLambertMaterial({ color: 0x0090b0 })
    );
    wingL.position.set(-9, 0, -1);
    droneGrp.add(wingL);
    var wingR = wingL.clone();
    wingR.position.set(9, 0, -1);
    droneGrp.add(wingR);
    droneMesh = droneGrp;
    droneMesh.position.set(pos.x, hbH + DRONE_ALT, pos.y);
    scene.add(droneMesh);

    // Trail
    trailLine = new THREE.Line(
        new THREE.BufferGeometry(),
        new THREE.LineBasicMaterial({ color: 0x00d4ff, transparent: true, opacity: 0.5 })
    );
    scene.add(trailLine);

    // Descent preview
    buildDescentPreview();

    // Camera
    camera.position.set(mapW / 2 + 300, 350, mapH / 2 + 300);
    controls.target.set(mapW / 2, 50, mapH / 2);
    controls.update();
}

// ============================================================
//  DESCENT PREVIEW
// ============================================================
function buildDescentPreview() {
    if (!missionData || !missionData.descent || !missionData.descent.waypoints || missionData.descent.waypoints.length < 2) return;
    var pts = missionData.descent.waypoints.map(function (w) {
        var bh = isLAC ? getTerrainHeight(w.x, w.y) : 0;
        return new THREE.Vector3(w.x, bh + (w.alt || 0), w.y);
    });
    var line = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(pts),
        new THREE.LineBasicMaterial({ color: 0xffcc00, transparent: true, opacity: 0.3 })
    );
    scene.add(line); descentMeshes.push(line);

    // Landing zone
    if (missionData.descent.center) {
        var cx = missionData.descent.center.x, cy = missionData.descent.center.y;
        var lr = missionData.descent.radius_m || 60;
        var lh = isLAC ? getTerrainHeight(cx, cy) : 0;
        var ring = new THREE.Mesh(
            new THREE.RingGeometry(lr * 0.3, lr * 0.35, 32),
            new THREE.MeshBasicMaterial({ color: 0xffcc00, side: THREE.DoubleSide, transparent: true, opacity: 0.25 })
        );
        ring.rotation.x = -Math.PI / 2;
        ring.position.set(cx, lh + 1, cy);
        scene.add(ring); descentMeshes.push(ring);

        var lbl = makeTextSprite('LANDING', '#ffcc00');
        lbl.position.set(cx, lh + 30, cy);
        scene.add(lbl); descentMeshes.push(lbl);
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

    var loiters = missionData.loiters || [];
    loiters.forEach(function (l) {
        safeWaypoints.push({ x: l.x, y: l.y, type: 'loiter', radius: l.radius || 60 });
    });

    // Return home
    safeWaypoints.push({ x: missionData.home.x, y: missionData.home.y, type: 'return' });
}

// ============================================================
//  COVERAGE
// ============================================================
function markCoverage(cx, cy, rad) {
    var res = (missionData && missionData.map && missionData.map.resolution) ? missionData.map.resolution : 10;
    var r2 = rad * rad;
    for (var dx = -rad; dx <= rad; dx += res) {
        for (var dy = -rad; dy <= rad; dy += res) {
            if (dx * dx + dy * dy > r2) continue;
            var gx = Math.floor((cx + dx) / res), gy = Math.floor((cy + dy) / res);
            var key = gx + ',' + gy;
            if (!coveredSet.has(key)) {
                coveredSet.add(key);
                var bH = isLAC ? getTerrainHeight(gx * res, gy * res) : 0;
                var covMesh = new THREE.Mesh(
                    new THREE.PlaneGeometry(res * 0.9, res * 0.9),
                    new THREE.MeshBasicMaterial({ color: 0x10b981, transparent: true, opacity: 0.2, side: THREE.DoubleSide })
                );
                covMesh.rotation.x = -Math.PI / 2;
                covMesh.position.set(gx * res, bH + 0.5, gy * res);
                scene.add(covMesh); coverageMeshes.push(covMesh);
            }
        }
    }
}

// ============================================================
//  SIMULATION
// ============================================================
function updateSim(dt) {
    if (paused || state === 'DONE' || state === 'IDLE' || !missionData) return;

    var powerW = missionData.performance ? missionData.performance.power_draw_w : 133;
    var cruiseSpeed = missionData.performance ? missionData.performance.cruise_speed_ms : 19;

    // DESCENT
    if (state === 'DESCENT') {
        if (descentWpIdx >= descentWps.length) { state = 'DONE'; currentAltAGL = 0; updateDroneMesh(); return; }
        var dw = descentWps[descentWpIdx];
        var dx = dw.x - pos.x, dy = dw.y - pos.y;
        var dist = Math.sqrt(dx * dx + dy * dy);
        if (dist < 15) { descentWpIdx++; }
        else {
            var th = Math.atan2(dy, dx);
            var spd = (dw.speed || cruiseSpeed) * 0.6;
            pos.x += spd * dt * Math.cos(th);
            pos.y += spd * dt * Math.sin(th);
            heading = th;
        }
        currentAltAGL = dw.alt || 0;
        distance += SPD * dt * 0.4;
        energyUsedWh += powerW * 0.6 * dt / 3600;
        battery = Math.max(0, (1 - energyUsedWh / energyCapacityWh) * 100);
        updateDroneMesh(); return;
    }

    // LOITER
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
        updateDroneMesh(); return;
    }

    // FLY
    if (wpIdx >= safeWaypoints.length) {
        if (missionData.descent && missionData.descent.waypoints && missionData.descent.waypoints.length > 0) {
            state = 'DESCENT'; descentWps = missionData.descent.waypoints; descentWpIdx = 0; currentAltAGL = DRONE_ALT;
        } else { state = 'DONE'; }
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
    pos.x += d * Math.cos(heading); pos.y += d * Math.sin(heading);
    distance += d;
    energyUsedWh += powerW * dt / 3600;
    battery = Math.max(0, (1 - energyUsedWh / energyCapacityWh) * 100);
    currentAltAGL = DRONE_ALT;
    markCoverage(pos.x, pos.y, isLAC ? 120 : 50);
    var captureR = (target.type === 'return' || target.type === 'home') ? 60 : (isLAC ? 50 : 20);
    if (distT < captureR) {
        if (target.type === 'loiter') {
            state = 'LOITER'; loiterCenter = { x: target.x, y: target.y };
            loiterAngle = heading - Math.PI / 2; loiterRevs = 0;
            loiterR = target.radius || 60;
        } else { wpIdx++; }
    }
    updateDroneMesh();
}

function updateDroneMesh() {
    if (!droneMesh) return;
    var baseH = isLAC ? getTerrainHeight(pos.x, pos.y) : 0;
    var alt = state === 'DESCENT' ? currentAltAGL : DRONE_ALT;
    droneMesh.position.set(pos.x, baseH + alt + Math.sin(simTime * 3) * (state === 'DESCENT' ? 0.5 : 3), pos.y);
    droneMesh.rotation.y = -heading + Math.PI / 2;
    droneMesh.rotation.z = (state === 'DESCENT' && currentAltAGL > 15) ? Math.sin(simTime * 2) * 0.3 : 0;

    var trailH = baseH + alt - 2;
    trailPoints.push(new THREE.Vector3(pos.x, trailH, pos.y));
    if (trailPoints.length > 4000) trailPoints.shift();
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
    var totalCells = Math.floor(mapW / res) * Math.floor(mapH / res);
    var covPct = Math.min(100, (coveredSet.size / totalCells) * 100);

    // Status
    var statusText = state === 'DONE' ? 'MISSION COMPLETE' :
        state === 'DESCENT' ? 'DESCENT' :
            state === 'LOITER' ? 'LOITERING' :
                state === 'FLY' ? 'EN ROUTE' : 'STANDBY';
    document.getElementById('hud-status').textContent = statusText;

    var phaseText = state === 'DONE' ? 'All loiters complete, landed' :
        state === 'DESCENT' ? 'Loiter-to-Land: ' + currentAltAGL.toFixed(0) + 'm AGL' :
            state === 'LOITER' ? 'Loiter ' + (nLoitersDone + 1) + '/' + totalLoiters :
                state === 'FLY' ? 'Transit to L' + (nLoitersDone + 1) : 'Awaiting mission';
    document.getElementById('hud-phase').textContent = phaseText;

    document.getElementById('hud-loiters').textContent = nLoitersDone + '/' + totalLoiters;
    document.getElementById('hud-distance').textContent = distance >= 1000 ? (distance / 1000).toFixed(1) + 'km' : distance.toFixed(0) + 'm';
    document.getElementById('hud-coverage').textContent = covPct.toFixed(1) + '%';

    // Bottom strip
    document.getElementById('hud-alt').textContent = (state === 'DESCENT' ? currentAltAGL.toFixed(0) : DRONE_ALT) + 'm';
    var cruiseSpd = missionData.performance ? missionData.performance.cruise_speed_ms : 18;
    document.getElementById('hud-spd').textContent = (state === 'DESCENT' ? (cruiseSpd * 0.6).toFixed(1) : cruiseSpd) + ' m/s';
    document.getElementById('hud-hdg').textContent = ((heading * 180 / Math.PI + 360) % 360).toFixed(0) + '\u00B0';
    var remainWh = Math.max(0, energyCapacityWh - energyUsedWh);
    document.getElementById('hud-energy').textContent = remainWh.toFixed(0) + ' Wh';
    document.getElementById('hud-bat').textContent = battery.toFixed(0) + '%';
    document.getElementById('hud-bat').className = 'hud-stat-value ' + (battery < 20 ? '' : battery < 40 ? 'amber' : 'green');
    document.getElementById('hud-state').textContent = state;
    document.getElementById('hud-speed').textContent = speed.toFixed(1) + 'x';

    // Update loiter mesh colors
    if (missionData.loiters) {
        missionData.loiters.forEach(function (l, i) {
            var mesh = loiterMeshes[i * 2];
            if (!mesh) return;
            if (i < nLoitersDone) { mesh.material.color.set(0x10b981); mesh.material.opacity = 0.7; }
            else if (i === nLoitersDone && state === 'LOITER') { mesh.material.color.set(0x00d4ff); mesh.material.opacity = 0.8; }
        });
    }
}

// ============================================================
//  ANIMATION LOOP
// ============================================================
function animate() {
    requestAnimationFrame(animate);
    var dt = speed * (1 / 60);
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
    document.getElementById('btnPlayPause').textContent = paused ? 'Play' : 'Pause';
}

function viewerSlower() { speed = Math.max(0.5, speed - 0.5); }
function viewerFaster() { speed = Math.min(10, speed + 0.5); }

function viewerReset() {
    if (!missionData) return;
    wpIdx = 0; state = 'FLY'; nLoitersDone = 0; distance = 0; battery = 100;
    energyUsedWh = 0; descentWpIdx = 0; currentAltAGL = DRONE_ALT;
    pos = { x: missionData.home.x, y: missionData.home.y };
    heading = 0; trailPoints = []; coveredSet.clear();
    coverageMeshes.forEach(function (m) { scene.remove(m); if (m.geometry) m.geometry.dispose(); if (m.material) m.material.dispose(); });
    coverageMeshes = [];
    loiterMeshes.forEach(function (m, i) {
        if (i % 2 === 0 && m.material) { m.material.color.set(0xffb040); m.material.opacity = 0.5; }
    });
    computeSafePath();
}

// ============================================================
//  MISSION PLANNER
// ============================================================
var plannerAltSlider = null;

function initPlanner() {
    plannerAltSlider = document.getElementById('plannerAlt');
    if (plannerAltSlider) {
        plannerAltSlider.addEventListener('input', function () {
            document.getElementById('plannerAltVal').textContent = this.value + 'm';
            updateISAPreview(parseFloat(this.value));
        });
        updateISAPreview(4000);
    }
}

async function updateISAPreview(alt) {
    try {
        var resp = await fetch('/api/performance/' + alt);
        if (!resp.ok) return;
        var data = await resp.json();
        document.getElementById('isaData').innerHTML =
            '<b>Cruise:</b> ' + data.cruise_speed_ms + ' m/s<br>' +
            '<b>Power:</b> ' + data.power_draw_w + ' W<br>' +
            '<b>Loiter R:</b> ' + data.loiter_radius_m + ' m<br>' +
            '<b>Density:</b> ' + data.air_density + ' kg/m\u00B3 (\u03C3=' + data.density_ratio + ')<br>' +
            '<b>Temp:</b> ' + data.temperature_c + '\u00B0C<br>' +
            '<b>Endurance:</b> ' + (data.endurance || '--') + ' hr';
    } catch (e) {
        document.getElementById('isaData').innerHTML = '<span class="text-muted">API unavailable (static mode)</span>';
    }
}

async function generateMission() {
    var btn = document.getElementById('btnGenerate');
    btn.textContent = 'Generating...';
    btn.disabled = true;

    var mapType = document.getElementById('plannerMap').value;
    var alt = parseFloat(document.getElementById('plannerAlt').value);
    var seedVal = document.getElementById('plannerSeed').value;
    var seed = seedVal ? parseInt(seedVal) : null;

    try {
        var resp = await fetch('/api/mission/generate', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ map_type: mapType, altitude_m: alt, seed: seed })
        });
        var data = await resp.json();

        if (data.success) {
            var resultDiv = document.getElementById('plannerResult');
            resultDiv.style.display = 'block';
            document.getElementById('plannerStats').innerHTML =
                '<b>Loiters:</b> ' + data.loiter_count + '<br>' +
                '<b>Energy:</b> ' + (data.energy.total_used_wh || 0).toFixed(1) + ' Wh<br>' +
                '<b>Remaining:</b> ' + (data.energy.remaining_pct || 0).toFixed(1) + '%<br>' +
                '<b>Descent:</b> ' + (data.descent.n_loops || 0) + ' loops<br>' +
                '<b>Speed:</b> ' + (data.performance.cruise_speed_ms || '--') + ' m/s';

            // Reload viewer data
            await loadMission();
            updateTelemetryPage();
        } else {
            alert('Generation failed: ' + JSON.stringify(data));
        }
    } catch (e) {
        alert('Error: ' + e.message);
    }

    btn.textContent = 'Generate Mission';
    btn.disabled = false;
}

// ============================================================
//  TELEMETRY PAGE
// ============================================================
function updateTelemetryPage() {
    if (!missionData) return;

    // Performance cards
    var perf = missionData.performance || {};
    document.getElementById('tel-speed').textContent = perf.cruise_speed_ms || '--';
    document.getElementById('tel-power').textContent = perf.power_draw_w || '--';
    document.getElementById('tel-radius').textContent = perf.loiter_radius_m || '--';
    document.getElementById('tel-density').textContent = perf.density_ratio || '--';

    // Energy donut
    drawEnergyDonut();

    // Phase timeline
    drawPhaseTimeline();

    // Mission summary
    var energy = missionData.energy || {};
    var descent = missionData.descent || {};
    document.getElementById('missionSummary').innerHTML =
        '<b>Loiters:</b> ' + (missionData.loiters ? missionData.loiters.length : 0) + '<br>' +
        '<b>Total Energy:</b> ' + (energy.total_used_wh || 0).toFixed(1) + ' Wh / ' + (energy.battery_capacity_wh || 370) + ' Wh<br>' +
        '<b>Remaining:</b> ' + (energy.remaining_pct || 0).toFixed(1) + '%<br>' +
        '<b>Duration:</b> ' + (energy.total_time_min || 0).toFixed(1) + ' min<br>' +
        '<b>Descent:</b> ' + (descent.n_loops || 0) + ' loops, ' + (descent.energy_wh || 0).toFixed(1) + ' Wh<br>' +
        '<b>Map:</b> ' + (missionData.map ? missionData.map.width + 'x' + missionData.map.height + 'm' : '--');
}

function drawEnergyDonut() {
    var canvas = document.getElementById('energyDonut');
    if (!canvas || !missionData) return;
    var ctx = canvas.getContext('2d');
    var cx = 140, cy = 140, outerR = 120, innerR = 80;

    ctx.clearRect(0, 0, 280, 280);

    var energy = missionData.energy || {};
    var phases = energy.phases || {};
    var total = energy.battery_capacity_wh || 370;
    var used = energy.total_used_wh || 0;
    var remaining = total - used;

    var slices = [
        { label: 'Climb', value: phases.climb_wh || 0, color: '#00d4ff' },
        { label: 'Transit', value: phases.transit_wh || 0, color: '#10b981' },
        { label: 'Loiter', value: phases.loiter_wh || 0, color: '#f59e0b' },
        { label: 'RTB', value: phases.rtb_wh || 0, color: '#a855f7' },
        { label: 'Descent', value: (missionData.descent ? missionData.descent.energy_wh : 0) || 0, color: '#ef4444' },
        { label: 'Reserve', value: Math.max(0, remaining), color: '#1e2a3a' },
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
    ctx.font = 'bold 28px Inter, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText(used.toFixed(0), cx, cy - 4);
    ctx.font = '12px Inter, sans-serif';
    ctx.fillStyle = '#94a3b8';
    ctx.fillText('Wh used / ' + total, cx, cy + 16);

    // Legend
    var legend = document.getElementById('energyLegend');
    if (legend) {
        legend.innerHTML = slices.filter(function (s) { return s.value > 0; }).map(function (s) {
            return '<span style="display:inline-flex;align-items:center;gap:6px;margin-right:14px">' +
                '<span style="width:10px;height:10px;border-radius:2px;background:' + s.color + ';display:inline-block"></span>' +
                s.label + ': ' + s.value.toFixed(1) + ' Wh</span>';
        }).join('');
    }
}

function drawPhaseTimeline() {
    var el = document.getElementById('phaseTimeline');
    if (!el || !missionData || !missionData.energy) return;
    var energy = missionData.energy;
    var phases = energy.phases || {};
    var total = energy.total_time_min || 1;

    var items = [
        { label: 'Climb', time: phases.climb_time_min || 0, color: '#00d4ff' },
        { label: 'Transit', time: phases.transit_time_min || 0, color: '#10b981' },
        { label: 'Loiter', time: phases.loiter_time_min || 0, color: '#f59e0b' },
        { label: 'RTB', time: phases.rtb_time_min || 0, color: '#a855f7' },
        { label: 'Descent', time: (missionData.descent ? missionData.descent.time_min : 0) || 0, color: '#ef4444' },
    ];

    var html = '<div style="display:flex;height:24px;border-radius:6px;overflow:hidden;margin-bottom:12px">';
    items.forEach(function (it) {
        var pct = (it.time / total) * 100;
        if (pct > 0.5) {
            html += '<div style="width:' + pct + '%;background:' + it.color + ';display:flex;align-items:center;justify-content:center;font-size:9px;font-weight:600;color:white;min-width:20px" title="' + it.label + ': ' + it.time.toFixed(1) + 'min">' + it.label + '</div>';
        }
    });
    html += '</div>';
    html += '<div style="display:flex;gap:16px;flex-wrap:wrap">';
    items.forEach(function (it) {
        html += '<span style="font-size:11px;color:var(--text-secondary)"><span style="display:inline-block;width:8px;height:8px;border-radius:2px;background:' + it.color + ';margin-right:4px"></span>' + it.label + ': ' + it.time.toFixed(1) + ' min</span>';
    });
    html += '</div>';
    el.innerHTML = html;
}

// ============================================================
//  EXPORTS PAGE
// ============================================================
async function updateExportsPage() {
    if (!missionData) return;
    var tbody = document.getElementById('statsBody');
    if (!tbody) return;

    var stats = [
        ['Map Type', missionData.map ? missionData.map.type : '--'],
        ['Map Size', missionData.map ? missionData.map.width + ' x ' + missionData.map.height + ' m' : '--'],
        ['Loiter Zones', missionData.loiters ? missionData.loiters.length : 0],
        ['Obstacles', missionData.obstacles ? missionData.obstacles.length : 0],
        ['Waypoints', missionData.waypoints ? missionData.waypoints.length : 0],
        ['Home', missionData.home ? '(' + missionData.home.x + ', ' + missionData.home.y + ')' : '--'],
    ];

    if (missionData.performance) {
        stats.push(['Cruise Speed', missionData.performance.cruise_speed_ms + ' m/s']);
        stats.push(['Power Draw', missionData.performance.power_draw_w + ' W']);
        stats.push(['Loiter Radius', missionData.performance.loiter_radius_m + ' m']);
        stats.push(['Density Ratio', missionData.performance.density_ratio + ' \u03C3']);
    }

    if (missionData.energy) {
        stats.push(['Total Energy', (missionData.energy.total_used_wh || 0).toFixed(1) + ' Wh']);
        stats.push(['Battery Capacity', (missionData.energy.battery_capacity_wh || 370) + ' Wh']);
        stats.push(['Remaining', (missionData.energy.remaining_pct || 0).toFixed(1) + '%']);
        stats.push(['Duration', (missionData.energy.total_time_min || 0).toFixed(1) + ' min']);
    }

    tbody.innerHTML = stats.map(function (s) {
        return '<tr><td>' + s[0] + '</td><td class="font-mono">' + s[1] + '</td></tr>';
    }).join('');

    // ISA table - try API first, fall back to hardcoded
    var isaBody = document.getElementById('isaTable');
    if (isaBody) {
        var altitudes = [0, 1000, 2000, 3000, 4000, 5000];
        try {
            var rows = '';
            for (var i = 0; i < altitudes.length; i++) {
                var resp = await fetch('/api/performance/' + altitudes[i]);
                if (resp.ok) {
                    var d = await resp.json();
                    rows += '<tr><td class="font-mono">' + altitudes[i] + 'm</td><td>' + d.cruise_speed_ms + '</td><td>' + d.power_draw_w + '</td><td>' + d.loiter_radius_m + '</td><td>' + d.stall_speed_ms + '</td></tr>';
                }
            }
            isaBody.innerHTML = rows;
        } catch (e) {
            isaBody.innerHTML = '<tr><td colspan="5" class="text-muted">API unavailable</td></tr>';
        }
    }
}

// ============================================================
//  INIT
// ============================================================
document.addEventListener('DOMContentLoaded', function () {
    initPlanner();
    initViewer();
});
