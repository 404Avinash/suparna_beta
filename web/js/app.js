/**
 * SUPARNA — Mission Control SPA v3
 * Merged from proven working index.html viewer + SPA framework
 */

// === API Configuration ===
// Use relative paths for all API calls so it works automatically on localhost and Render
var API_BASE_URL = '';

// === Error overlay ===
window.onerror = function (msg, url, line) {
    var o = document.getElementById('_dbg') || (function () {
        var d = document.createElement('div'); d.id = '_dbg';
        d.style.cssText = 'position:fixed;top:0;left:72px;right:0;z-index:9999;background:rgba(220,38,38,.95);color:#fff;font:12px monospace;padding:10px;max-height:150px;overflow:auto';
        document.body.appendChild(d); return d;
    })();
    o.innerHTML += '<div>ERR: ' + msg + ' at line ' + line + '</div>';
};

// === Globals ===
var scene, camera, renderer, controls;
var missionData = null;
var droneMesh, trailLine, trailPoints = [];
var loiterMeshes = [], obstacleMeshes = [], coverageMeshes = [], labelSprites = [];
var groundPlane, gridHelper, terrainMesh;
var descentMeshes = [], allSceneObjects = [];

var simTime = 0, speed = 2.0, paused = false;
var wpIdx = 0, state = 'FLY';
var pos = { x: 0, y: 0 }, heading = 0;
var loiterCenter = null, loiterAngle = 0, loiterRevs = 0, loiterR = 60;
var nLoitersDone = 0, distance = 0, battery = 100;
var coveredSet = new Set();
var safeWaypoints = [];
var isLAC = false;
var SPD = 35, TURN_RATE = 2.5, DRONE_ALT = 15;
var energyUsedWh = 0, energyCapacityWh = 370;
var descentWpIdx = 0, descentWps = [];
var currentAltAGL = 0;
var viewerInitialized = false;
var animRunning = false;

// === Restricted Zones (Click-to-Restrict) ===
var customObstacles = [];      // { x, y, radius, mesh, ring }
var customObstMeshes = [];     // all Three.js objects for custom zones
var restrictMode = false;
var raycaster = null;
var mouseVec = null;
var mapGroundRef = null;       // reference to the clickable ground plane
var RESTRICT_RADIUS = 60;      // default radius for restricted zone
var missionStarted = false;    // true after operator clicks START

// ============================================================
//  NAVIGATION
// ============================================================
function navigate(page) {
    document.querySelectorAll('.page').forEach(function (p) { p.classList.remove('active') });
    document.querySelectorAll('.nav-item').forEach(function (n) { n.classList.remove('active') });
    var el = document.getElementById('page-' + page);
    if (el) el.classList.add('active');
    var nav = document.querySelector('.nav-item[data-page="' + page + '"]');
    if (nav) nav.classList.add('active');
    if (page === 'viewer' && !viewerInitialized) initViewer();
    if (page === 'viewer' && renderer) onResize();
    if (page === 'planner' && leafletMap) setTimeout(function () { leafletMap.invalidateSize(); }, 200);
    if (page === 'telemetry') updateTelemetryPage();
    if (page === 'exports') updateExportsPage();
}

// ============================================================
//  CLOCK
// ============================================================
function updateClock() {
    var d = new Date();
    var el = document.getElementById('clock');
    if (el) el.textContent = String(d.getHours()).padStart(2, '0') + ':' + String(d.getMinutes()).padStart(2, '0') + ':' + String(d.getSeconds()).padStart(2, '0');
}
setInterval(updateClock, 1000); updateClock();

// ============================================================
//  THREE.JS INIT
// ============================================================
function onResize() {
    var c = document.getElementById('viewer-canvas-wrap');
    if (!c || !renderer) return;
    var w = c.clientWidth, h = c.clientHeight;
    if (w < 10 || h < 10) return;
    camera.aspect = w / h; camera.updateProjectionMatrix();
    renderer.setSize(w, h);
}

function initViewer() {
    var container = document.getElementById('viewer-canvas-wrap');
    if (!container) return;

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0a0e17);

    var w = container.clientWidth || window.innerWidth - 72;
    var h = container.clientHeight || window.innerHeight - 56;

    camera = new THREE.PerspectiveCamera(50, w / h, 1, 20000);
    camera.position.set(600, 500, 600);

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(w, h);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    container.appendChild(renderer.domElement);

    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true; controls.dampingFactor = 0.08;
    controls.maxPolarAngle = Math.PI / 2.1;

    scene.add(new THREE.AmbientLight(0x334466, 0.6));
    var dl = new THREE.DirectionalLight(0xffffff, 0.8);
    dl.position.set(2000, 3000, 1500); dl.castShadow = true;
    dl.shadow.camera.near = 1; dl.shadow.camera.far = 10000;
    dl.shadow.camera.left = -3000; dl.shadow.camera.right = 3000;
    dl.shadow.camera.top = 3000; dl.shadow.camera.bottom = -3000;
    dl.shadow.mapSize.set(2048, 2048);
    scene.add(dl);
    scene.add(new THREE.HemisphereLight(0x88aacc, 0x443322, 0.4));

    window.addEventListener('resize', onResize);
    viewerInitialized = true;

    // Raycaster for click-to-restrict
    raycaster = new THREE.Raycaster();
    mouseVec = new THREE.Vector2();
    renderer.domElement.addEventListener('click', onViewerClick);
    renderer.domElement.addEventListener('contextmenu', onViewerRightClick);

    if (!animRunning) { animRunning = true; animate(); }
    loadMission();
}

// ============================================================
//  TERRAIN
// ============================================================
function getTerrainColor(elevation, minE, maxE) {
    var t = (elevation - minE) / (maxE - minE);
    var r, g, b;
    // Subtle dark tones — satellite overlay is the primary visual
    if (t < 0.2) { r = 0.10 + t * 0.1; g = 0.10 + t * 0.1; b = 0.08 + t * 0.05; }
    else if (t < 0.45) { var s = (t - 0.2) / 0.25; r = 0.12 + s * 0.08; g = 0.12 + s * 0.06; b = 0.10 + s * 0.06; }
    else if (t < 0.7) { var s2 = (t - 0.45) / 0.25; r = 0.20 + s2 * 0.06; g = 0.18 + s2 * 0.06; b = 0.16 + s2 * 0.08; }
    else { var s3 = (t - 0.7) / 0.3; r = 0.26 + s3 * 0.15; g = 0.24 + s3 * 0.18; b = 0.24 + s3 * 0.20; }
    return new THREE.Color(r, g, b);
}

function buildTerrain(data) {
    var hm = data.heightmap;
    var rows = hm.rows, cols = hm.cols;
    var W = data.map.width, H = data.map.height;
    var minE = hm.min_elevation, maxE = hm.max_elevation;
    var elevScale = 0.15;

    var geo = new THREE.PlaneGeometry(W, H, cols - 1, rows - 1);
    geo.rotateX(-Math.PI / 2);
    var positions = geo.attributes.position.array;
    var colors = new Float32Array(positions.length);

    for (var r2 = 0; r2 < rows; r2++) {
        for (var c = 0; c < cols; c++) {
            var idx = r2 * cols + c;
            var vi = idx * 3;
            var elev = hm.data[idx] || 0;
            positions[vi + 1] = (elev - minE) * elevScale;
            var clr = getTerrainColor(elev, minE, maxE);
            colors[vi] = clr.r; colors[vi + 1] = clr.g; colors[vi + 2] = clr.b;
        }
    }
    geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    geo.computeVertexNormals();

    var mat = new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.85, metalness: 0.05, flatShading: true });
    terrainMesh = new THREE.Mesh(geo, mat);
    terrainMesh.position.set(W / 2, 0, H / 2);
    terrainMesh.receiveShadow = true;
    scene.add(terrainMesh);

    // Satellite map overlay on top of terrain
    loadSatelliteOverlay(W, H, 2);

    return elevScale;
}

// ============================================================
//  SATELLITE MAP OVERLAY
// ============================================================
function loadSatelliteOverlay(W, H, yOffset) {
    // Get coordinates from mission data or defaults
    var coords = (missionData && missionData.coordinates) || {};
    var lat = coords.latitude || 34.1526;
    var lon = coords.longitude || 77.5771;
    // Calculate bounding box (~4km span)
    var span = 0.04;
    var west = lon - span, south = lat - span, east = lon + span, north = lat + span;

    // Use ESRI World Imagery static export API for satellite image
    var url = 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/export' +
        '?bbox=' + west + ',' + south + ',' + east + ',' + north +
        '&bboxSR=4326&imageSR=4326&size=1024,1024&format=png&f=image';

    var texLoader = new THREE.TextureLoader();
    texLoader.crossOrigin = 'anonymous';
    texLoader.load(url, function (texture) {
        texture.minFilter = THREE.LinearFilter;
        texture.magFilter = THREE.LinearFilter;
        var overlayGeo = new THREE.PlaneGeometry(W, H);
        var overlayMat = new THREE.MeshBasicMaterial({
            map: texture,
            transparent: true,
            opacity: 0.75,
            depthWrite: false,
            side: THREE.DoubleSide,
        });
        var overlayMesh = new THREE.Mesh(overlayGeo, overlayMat);
        overlayMesh.rotation.x = -Math.PI / 2;
        overlayMesh.position.set(W / 2, yOffset || 2, H / 2);
        scene.add(overlayMesh);
        allSceneObjects.push(overlayMesh);
    }, undefined, function () {
        // Fallback: use CartoDB dark tiles canvas
        loadCartoDarkOverlay(W, H, lat, lon, yOffset);
    });
}

function loadCartoDarkOverlay(W, H, lat, lon, yOffset) {
    // Fallback: create a canvas with CartoDB tiles
    var canvas = document.createElement('canvas');
    canvas.width = 1024; canvas.height = 1024;
    var ctx = canvas.getContext('2d');
    // Dark base
    ctx.fillStyle = '#1a1a2e';
    ctx.fillRect(0, 0, 1024, 1024);
    // Grid lines for map feel
    ctx.strokeStyle = 'rgba(0,212,255,0.08)';
    ctx.lineWidth = 1;
    for (var i = 0; i < 1024; i += 64) {
        ctx.beginPath(); ctx.moveTo(i, 0); ctx.lineTo(i, 1024); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(0, i); ctx.lineTo(1024, i); ctx.stroke();
    }
    // Coordinate labels
    ctx.fillStyle = 'rgba(0,212,255,0.15)';
    ctx.font = '14px JetBrains Mono, monospace';
    ctx.fillText(lat.toFixed(3) + '°N', 20, 30);
    ctx.fillText(lon.toFixed(3) + '°E', 20, 50);
    ctx.fillText('SATELLITE OVERLAY', 20, 1010);

    var texture = new THREE.CanvasTexture(canvas);
    var overlayGeo = new THREE.PlaneGeometry(W, H);
    var overlayMat = new THREE.MeshBasicMaterial({
        map: texture,
        transparent: true,
        opacity: 0.3,
        depthWrite: false,
        side: THREE.DoubleSide,
    });
    var overlayMesh = new THREE.Mesh(overlayGeo, overlayMat);
    overlayMesh.rotation.x = -Math.PI / 2;
    overlayMesh.position.set(W / 2, yOffset || 2, H / 2);
    scene.add(overlayMesh);
    allSceneObjects.push(overlayMesh);
}

function getTerrainHeight(px, py) {
    if (!missionData || !missionData.heightmap) return 0;
    var hm = missionData.heightmap;
    var step = hm.step || 40;
    var c = Math.floor(px / step), r = Math.floor(py / step);
    c = Math.max(0, Math.min(c, hm.cols - 1));
    r = Math.max(0, Math.min(r, hm.rows - 1));
    var elev = hm.data[r * hm.cols + c] || 0;
    return (elev - hm.min_elevation) * 0.15;
}

// ============================================================
//  TEXT SPRITE
// ============================================================
function makeTextSprite(text, color) {
    var canvas = document.createElement('canvas');
    canvas.width = 512; canvas.height = 128;
    var ctx = canvas.getContext('2d');
    ctx.font = 'bold 36px Segoe UI, Inter, sans-serif';
    ctx.fillStyle = color || '#ffffff';
    ctx.textAlign = 'center';
    ctx.fillText(text, 256, 70);
    var tex = new THREE.CanvasTexture(canvas);
    var mat = new THREE.SpriteMaterial({ map: tex, transparent: true, opacity: 0.85 });
    var sprite = new THREE.Sprite(mat);
    sprite.scale.set(200, 50, 1);
    return sprite;
}

// ============================================================
//  BUILD SCENE (from proven working code)
// ============================================================
function buildScene(data) {
    if (!data || !scene) return;
    // Clear
    [].concat(loiterMeshes, obstacleMeshes, coverageMeshes, labelSprites, descentMeshes, allSceneObjects).forEach(function (m) { scene.remove(m) });
    if (groundPlane) scene.remove(groundPlane);
    if (gridHelper) scene.remove(gridHelper);
    if (terrainMesh) { scene.remove(terrainMesh); terrainMesh = null; }
    if (droneMesh) scene.remove(droneMesh);
    if (trailLine) scene.remove(trailLine);
    loiterMeshes = []; obstacleMeshes = []; coverageMeshes = []; labelSprites = []; descentMeshes = []; allSceneObjects = [];

    var W = data.map.width, H = data.map.height;
    isLAC = data.map.type === 'lac';

    if (isLAC && data.heightmap) {
        scene.background = new THREE.Color(0x0b1020);
        scene.fog = new THREE.Fog(0x1a2540, 2000, 8000);
        SPD = 60; DRONE_ALT = 80;
        buildTerrain(data);
        mapGroundRef = terrainMesh;
    } else {
        scene.background = new THREE.Color(0x0a0e17);
        scene.fog = new THREE.Fog(0x0a0e17, 800, 1800);
        SPD = 35; DRONE_ALT = 15;
        groundPlane = new THREE.Mesh(
            new THREE.PlaneGeometry(W, H),
            new THREE.MeshStandardMaterial({ color: 0x161a24, roughness: 0.9 })
        );
        groundPlane.rotation.x = -Math.PI / 2;
        groundPlane.position.set(W / 2, -0.5, H / 2);
        groundPlane.receiveShadow = true; scene.add(groundPlane);
        gridHelper = new THREE.GridHelper(Math.max(W, H), 40, 0x1a2030, 0x141820);
        gridHelper.position.set(W / 2, 0, H / 2); scene.add(gridHelper);
        // Satellite overlay for random maps too
        loadSatelliteOverlay(W, H, 0.5);
        mapGroundRef = groundPlane;
    }

    // Re-render custom restricted zones
    customObstMeshes.forEach(function (m) { scene.remove(m); });
    customObstMeshes = [];
    customObstacles.forEach(function (co) {
        addCustomObstMeshAt(co.x, co.y, co.radius);
    });

    // (Pre-built map obstacles are hidden — the user marks their own restricted zones)

    // Landmarks (dict format) — always show for context
    if (isLAC && data.landmarks) {
        var lmKeys = Array.isArray(data.landmarks) ? null : Object.keys(data.landmarks);
        if (lmKeys) {
            lmKeys.forEach(function (name) {
                var lm = data.landmarks[name];
                var bH = getTerrainHeight(lm.x, lm.y);
                var lbl = makeTextSprite(name, '#00d4ff');
                lbl.position.set(lm.x, bH + 60, lm.y); lbl.scale.set(300, 75, 1);
                scene.add(lbl); labelSprites.push(lbl);
                var pin = new THREE.Mesh(new THREE.SphereGeometry(8, 8, 8), new THREE.MeshBasicMaterial({ color: 0x00d4ff, transparent: true, opacity: 0.6 }));
                pin.position.set(lm.x, bH + 40, lm.y);
                scene.add(pin); labelSprites.push(pin);
            });
        } else {
            data.landmarks.forEach(function (lm) {
                var bH = getTerrainHeight(lm.x, lm.y);
                var lbl = makeTextSprite(lm.name, '#00d4ff');
                lbl.position.set(lm.x, bH + 60, lm.y); lbl.scale.set(300, 75, 1);
                scene.add(lbl); labelSprites.push(lbl);
                var pin = new THREE.Mesh(new THREE.SphereGeometry(8, 8, 8), new THREE.MeshBasicMaterial({ color: 0x00d4ff, transparent: true, opacity: 0.6 }));
                pin.position.set(lm.x, bH + 40, lm.y);
                scene.add(pin); labelSprites.push(pin);
            });
        }
    }

    // Home marker — always visible
    var homeH = isLAC ? getTerrainHeight(data.home.x, data.home.y) : 0;
    var hm2 = new THREE.Mesh(
        new THREE.ConeGeometry(isLAC ? 15 : 8, isLAC ? 35 : 20, 6),
        new THREE.MeshStandardMaterial({ color: 0x1db954, emissive: 0x0a5020 })
    );
    hm2.position.set(data.home.x, homeH + 17, data.home.y);
    scene.add(hm2); obstacleMeshes.push(hm2);
    if (isLAC) {
        var homeLbl = makeTextSprite('INDIAN FOB (HOME)', '#1db954');
        homeLbl.position.set(data.home.x, homeH + 60, data.home.y);
        scene.add(homeLbl); labelSprites.push(homeLbl);
    }

    // === Mission elements — only shown after START ===
    if (missionStarted) {
        // Loiter targets (tactical ground rings + vertical lines)
        (data.loiters || []).forEach(function (l) {
            var baseH2 = isLAC ? getTerrainHeight(l.x, l.y) : 0;
            var torusH = baseH2 + DRONE_ALT;

            // Ground tactical ring (shows the exact orbit on the ground)
            var groundRing = new THREE.Mesh(
                new THREE.RingGeometry(l.radius - 2, l.radius, 64),
                new THREE.MeshBasicMaterial({ color: 0xff3366, transparent: true, opacity: 0.4, side: THREE.DoubleSide })
            );
            groundRing.rotation.x = -Math.PI / 2;
            groundRing.position.set(l.x, baseH2 + 2, l.y);
            scene.add(groundRing); loiterMeshes.push(groundRing);

            // Floating drone altitude ring
            var torus = new THREE.Mesh(
                new THREE.TorusGeometry(l.radius, isLAC ? 3 : 1.5, 8, 48),
                new THREE.MeshBasicMaterial({ color: 0xffb040, transparent: true, opacity: 0.6 })
            );
            torus.rotation.x = -Math.PI / 2; torus.position.set(l.x, torusH, l.y);
            scene.add(torus); loiterMeshes.push(torus);

            // Vertical beam
            var vl = new THREE.Line(
                new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(l.x, baseH2, l.y), new THREE.Vector3(l.x, torusH, l.y)]),
                new THREE.LineBasicMaterial({ color: 0xffb040, transparent: true, opacity: 0.3 })
            );
            scene.add(vl); loiterMeshes.push(vl);
        });

        // Drone
        var dg = new THREE.Group();
        var body = new THREE.Mesh(
            new THREE.ConeGeometry(isLAC ? 10 : 6, isLAC ? 30 : 18, 4),
            new THREE.MeshStandardMaterial({ color: 0x00d4ff, emissive: 0x004466, metalness: 0.5, roughness: 0.3 })
        );
        body.rotation.z = Math.PI / 2; body.rotation.y = Math.PI / 2; dg.add(body);
        var glow = new THREE.Mesh(
            new THREE.RingGeometry(isLAC ? 16 : 10, isLAC ? 22 : 14, 20),
            new THREE.MeshBasicMaterial({ color: 0x00d4ff, transparent: true, opacity: 0.15, side: THREE.DoubleSide })
        );
        glow.rotation.x = -Math.PI / 2; dg.add(glow);
        var navL = new THREE.Mesh(new THREE.SphereGeometry(1.5, 6, 6), new THREE.MeshBasicMaterial({ color: 0xff0040 }));
        navL.position.set(isLAC ? -18 : -10, 0, 0); dg.add(navL);
        var navR = new THREE.Mesh(new THREE.SphereGeometry(1.5, 6, 6), new THREE.MeshBasicMaterial({ color: 0x00ff40 }));
        navR.position.set(isLAC ? 18 : 10, 0, 0); dg.add(navR);
        droneMesh = dg;
        droneMesh.position.set(data.home.x, homeH + DRONE_ALT, data.home.y);
        scene.add(droneMesh);

        // Trail (what's already flown)
        trailLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x00d4ff, transparent: true, opacity: 0.8, linewidth: 2 }));
        scene.add(trailLine);

        // Path preview (Vibrant glowing line for the upcoming planned path)
        var pts = safeWaypoints.map(function (w) {
            var bh = isLAC ? getTerrainHeight(w.x, w.y) : 0;
            return new THREE.Vector3(w.x, bh + DRONE_ALT - 2, w.y);
        });
        if (pts.length > 1) {
            // Laid-ahead glowing trajectory
            var pl = new THREE.Line(
                new THREE.BufferGeometry().setFromPoints(pts),
                new THREE.LineBasicMaterial({ color: 0x00e5ff, transparent: true, opacity: 0.65 })
            );
            scene.add(pl); allSceneObjects.push(pl);
        }

        // Descent preview
        buildDescentPreview(data);
    }

    // Camera
    controls.target.set(W / 2, isLAC ? 100 : 0, H / 2);
    var camDist = Math.max(W, H) * (isLAC ? 1.2 : 0.9);
    camera.position.set(W / 2, camDist * 0.6, H / 2 + camDist * 0.5);
    controls.minDistance = isLAC ? 500 : 200;
    controls.maxDistance = isLAC ? 10000 : 2000;
    controls.update();
}

function buildDescentPreview(data) {
    if (!data.descent || !data.descent.waypoints || data.descent.waypoints.length < 2) return;
    var pts = data.descent.waypoints.map(function (w) {
        var bh = isLAC ? getTerrainHeight(w.x, w.y) : 0;
        return new THREE.Vector3(w.x, bh + (w.alt || 0), w.y);
    });
    var line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: 0xffcc00, transparent: true, opacity: 0.4 }));
    scene.add(line); descentMeshes.push(line);
    if (data.descent.center) {
        var lastWp = data.descent.waypoints[data.descent.waypoints.length - 1];
        var landH = isLAC ? getTerrainHeight(lastWp.x, lastWp.y) : 0;
        var lr = data.descent.radius_m || 60;
        var ring = new THREE.Mesh(new THREE.RingGeometry(lr * 0.3, lr * 0.35, 32), new THREE.MeshBasicMaterial({ color: 0xffcc00, side: THREE.DoubleSide, transparent: true, opacity: 0.3 }));
        ring.rotation.x = -Math.PI / 2; ring.position.set(data.descent.center.x, landH + 1, data.descent.center.y);
        scene.add(ring); descentMeshes.push(ring);
        var lbl = makeTextSprite('LANDING ZONE', '#ffcc00');
        lbl.position.set(data.descent.center.x, landH + 30, data.descent.center.y);
        scene.add(lbl); descentMeshes.push(lbl);
    }
}

// ============================================================
//  SAFE PATH (with obstacle avoidance from working code)
// ============================================================
function ptInObs(px, py) {
    // Check custom obstacles (user-placed restricted zones)
    for (var j = 0; j < customObstacles.length; j++) {
        var co = customObstacles[j];
        var dx2 = px - co.x, dy2 = py - co.y;
        if (Math.sqrt(dx2 * dx2 + dy2 * dy2) < co.radius + 35) return true;
    }
    return false;
}
function lineHitsObs(x1, y1, x2, y2) {
    // Check custom obstacles (user-placed restricted zones)
    for (var j = 0; j < customObstacles.length; j++) {
        var co = customObstacles[j];
        var margin = co.radius + 40;
        var dx3 = x2 - x1, dy3 = y2 - y1, len3 = dx3 * dx3 + dy3 * dy3;
        if (len3 < 1) continue;
        var t2 = ((co.x - x1) * dx3 + (co.y - y1) * dy3) / len3;
        t2 = Math.max(0, Math.min(1, t2));
        var px3 = x1 + t2 * dx3, py3 = y1 + t2 * dy3;
        var dist2 = Math.sqrt((co.x - px3) * (co.x - px3) + (co.y - py3) * (co.y - py3));
        if (dist2 < margin) return co;
    }
    return null;
}
function computeSafePath() {
    if (!missionData) return;
    var raw = missionData.waypoints || [];
    safeWaypoints = [];
    for (var i = 0; i < raw.length; i++) {
        safeWaypoints.push(raw[i]);
        if (i < raw.length - 1) {
            for (var attempt = 0; attempt < 6; attempt++) {
                var last = safeWaypoints[safeWaypoints.length - 1];
                var obs = lineHitsObs(last.x, last.y, raw[i + 1].x, raw[i + 1].y);
                if (!obs) break;
                var dx = raw[i + 1].x - last.x, dy = raw[i + 1].y - last.y;
                var len = Math.sqrt(dx * dx + dy * dy) || 1;
                var ppx = -dy / len, ppy = dx / len;
                var detourR = obs.radius + 55;
                var d1x = obs.x + ppx * detourR, d1y = obs.y + ppy * detourR;
                var d2x = obs.x - ppx * detourR, d2y = obs.y - ppy * detourR;
                var detour;
                if (!ptInObs(d1x, d1y)) detour = { x: d1x, y: d1y, type: 'detour' };
                else if (!ptInObs(d2x, d2y)) detour = { x: d2x, y: d2y, type: 'detour' };
                else detour = { x: obs.x + ppx * (detourR + 50), y: obs.y + ppy * (detourR + 50), type: 'detour' };
                safeWaypoints.push(detour);
            }
        }
    }
}
function normalizeAngle(a) { while (a > Math.PI) a -= 2 * Math.PI; while (a < -Math.PI) a += 2 * Math.PI; return a; }

// ============================================================
//  SIMULATION
// ============================================================
function updateSim(dt) {
    if (paused || state === 'DONE' || !missionData) return;
    var powerW = missionData.performance ? missionData.performance.power_draw_w : 133;
    var cruiseSpeed = missionData.performance ? missionData.performance.cruise_speed_ms : 19;

    if (state === 'DESCENT') {
        if (descentWpIdx >= descentWps.length) { state = 'DONE'; currentAltAGL = 0; updateDroneMesh(); return; }
        var dw = descentWps[descentWpIdx];
        var dx = dw.x - pos.x, dy = dw.y - pos.y;
        var dist = Math.sqrt(dx * dx + dy * dy);
        if (dist < 15) { descentWpIdx++; }
        else { var th = Math.atan2(dy, dx); var spd = (dw.speed || cruiseSpeed) * 0.6; pos.x += spd * dt * Math.cos(th); pos.y += spd * dt * Math.sin(th); heading = th; }
        currentAltAGL = dw.alt || 0;
        distance += SPD * dt * 0.4;
        energyUsedWh += powerW * 0.6 * dt / 3600;
        battery = Math.max(0, (1 - energyUsedWh / energyCapacityWh) * 100);
        updateDroneMesh(); return;
    }

    if (state === 'LOITER') {
        if (!loiterCenter) { state = 'FLY'; return; }
        var w = SPD / loiterR;
        loiterAngle += w * dt; loiterRevs += w * dt / (2 * Math.PI);
        pos.x = loiterCenter.x + loiterR * Math.cos(loiterAngle);
        pos.y = loiterCenter.y + loiterR * Math.sin(loiterAngle);
        heading = normalizeAngle(loiterAngle + Math.PI / 2);
        distance += SPD * dt;
        energyUsedWh += powerW * 0.92 * dt / 3600;
        battery = Math.max(0, (1 - energyUsedWh / energyCapacityWh) * 100);
        currentAltAGL = DRONE_ALT;
        markCoverage(loiterCenter.x, loiterCenter.y, loiterR + 30);
        if (loiterRevs >= 1) { nLoitersDone++; state = 'FLY'; loiterCenter = null; wpIdx++; }
        updateDroneMesh(); return;
    }

    if (wpIdx >= safeWaypoints.length) {
        if (missionData.descent && missionData.descent.waypoints && missionData.descent.waypoints.length > 0) {
            state = 'DESCENT'; descentWps = missionData.descent.waypoints; descentWpIdx = 0; currentAltAGL = DRONE_ALT;
        } else { state = 'DONE'; }
        return;
    }
    var target = safeWaypoints[wpIdx];
    var distT = Math.sqrt((target.x - pos.x) * (target.x - pos.x) + (target.y - pos.y) * (target.y - pos.y));
    var th2 = Math.atan2(target.y - pos.y, target.x - pos.x);
    var err = normalizeAngle(th2 - heading);
    var mt = TURN_RATE * dt;
    if (Math.abs(err) > mt) heading += err > 0 ? mt : -mt; else heading = th2;
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

function markCoverage(cx, cy, radius) {
    if (!missionData) return;
    var res = missionData.map.resolution || 10;
    var gcx = Math.floor(cx / res), gcy = Math.floor(cy / res);
    var rc = Math.ceil(radius / res);
    for (var dx = -rc; dx <= rc; dx++) {
        for (var dy = -rc; dy <= rc; dy++) {
            if (dx * dx + dy * dy > rc * rc) continue;
            var nx = gcx + dx, ny = gcy + dy, key = nx + ',' + ny;
            if (coveredSet.has(key)) continue;
            var wx = nx * res, wy = ny * res;
            if (wx < 0 || wx >= missionData.map.width || wy < 0 || wy >= missionData.map.height) continue;
            if (ptInObs(wx, wy)) continue;
            coveredSet.add(key);
            var baseH = isLAC ? getTerrainHeight(wx, wy) : 0;
            var tile = new THREE.Mesh(
                new THREE.PlaneGeometry(res, res),
                new THREE.MeshBasicMaterial({ color: isLAC ? 0x00aa44 : 0x1db954, transparent: true, opacity: isLAC ? 0.25 : 0.35, side: THREE.DoubleSide })
            );
            tile.rotation.x = -Math.PI / 2;
            tile.position.set(wx + res / 2, baseH + 0.5, wy + res / 2);
            scene.add(tile); coverageMeshes.push(tile);
        }
    }
}

function updateDroneMesh() {
    if (!droneMesh) return;
    var baseH = isLAC ? getTerrainHeight(pos.x, pos.y) : 0;
    var alt = state === 'DESCENT' ? currentAltAGL : DRONE_ALT;
    droneMesh.position.set(pos.x, baseH + alt + Math.sin(simTime * 3) * (state === 'DESCENT' ? 0.5 : 3), pos.y);
    droneMesh.rotation.y = -heading + Math.PI / 2;
    if (state === 'DESCENT' && currentAltAGL > 15) droneMesh.rotation.z = Math.sin(simTime * 2) * 0.3;
    else if (state === 'LOITER') droneMesh.rotation.z = 0.35;
    else droneMesh.rotation.z *= 0.9;
    var trailH = baseH + alt - 2;
    trailPoints.push(new THREE.Vector3(pos.x, trailH, pos.y));
    if (trailPoints.length > 3000) trailPoints.shift();
    if (trailLine && trailPoints.length > 1) {
        trailLine.geometry.dispose();
        trailLine.geometry = new THREE.BufferGeometry().setFromPoints(trailPoints);
    }
}

// ============================================================
//  HUD UPDATE
// ============================================================
function setHUD(id, val) { var el = document.getElementById('hud-' + id); if (el) el.textContent = val; }

function updateHUD() {
    if (!missionData) return;
    var totalLoiters = missionData.loiters ? missionData.loiters.length : 0;
    var W = missionData.map.width, H = missionData.map.height;
    var res = missionData.map.resolution || 10;
    var totalCells = Math.floor(W / res) * Math.floor(H / res);
    var covPct = Math.min(100, (coveredSet.size / totalCells) * 100);

    var statusText = state === 'DONE' ? 'MISSION COMPLETE' :
        state === 'DESCENT' ? 'LOITER-TO-LAND (' + currentAltAGL.toFixed(0) + 'm AGL)' :
            state === 'LOITER' ? 'LOITERING (' + (nLoitersDone + 1) + '/' + totalLoiters + ')' :
                wpIdx >= safeWaypoints.length - 1 ? 'RETURNING HOME' : 'EN ROUTE > L' + (nLoitersDone + 1);
    setHUD('status', statusText);
    setHUD('phase', state === 'DONE' ? 'All loiters complete. Touchdown.' :
        state === 'DESCENT' ? 'Spiral descent' : 'Surveillance mission active');
    setHUD('loiters', nLoitersDone + '/' + totalLoiters);
    setHUD('distance', distance >= 1000 ? (distance / 1000).toFixed(1) + ' km' : distance.toFixed(0) + ' m');
    setHUD('coverage', covPct.toFixed(1) + '%');

    var cruiseSpd = missionData.performance ? missionData.performance.cruise_speed_ms : 18;
    setHUD('alt', (state === 'DESCENT' ? currentAltAGL.toFixed(0) : DRONE_ALT) + 'm');
    setHUD('spd', (state === 'DESCENT' ? (cruiseSpd * 0.5).toFixed(1) : cruiseSpd) + ' m/s');
    setHUD('hdg', ((heading * 180 / Math.PI + 360) % 360).toFixed(0) + '\u00B0');
    var remainWh = Math.max(0, energyCapacityWh - energyUsedWh);
    setHUD('energy', remainWh.toFixed(0) + ' Wh');

    var batEl = document.getElementById('hud-bat');
    if (batEl) { batEl.textContent = battery.toFixed(0) + '%'; batEl.className = 'hud-stat-value ' + (battery < 20 ? '' : battery < 40 ? 'amber' : 'green'); }
    var stateEl = document.getElementById('hud-state');
    if (stateEl) { stateEl.textContent = state; stateEl.className = 'hud-stat-value ' + (state === 'DONE' ? 'green' : state === 'DESCENT' ? 'amber' : 'cyan'); }
    var speedEl = document.getElementById('hud-speed');
    if (speedEl) speedEl.textContent = speed.toFixed(1) + 'x';

    // Loiter color updates
    (missionData.loiters || []).forEach(function (l, i) {
        var groundRing = loiterMeshes[i * 3];
        var torus = loiterMeshes[i * 3 + 1];
        var vl = loiterMeshes[i * 3 + 2];
        if (!torus) return;

        if (i < nLoitersDone) {
            torus.material.color.set(0x1db954); torus.material.opacity = 0.7;
            groundRing.material.color.set(0x1db954); groundRing.material.opacity = 0.2;
            vl.material.color.set(0x1db954);
        } else if (i === nLoitersDone && state === 'LOITER') {
            torus.material.color.set(0x00d4ff); torus.material.opacity = 0.8;
            groundRing.material.color.set(0x00d4ff); groundRing.material.opacity = 0.5;
            vl.material.color.set(0x00d4ff);
        }
    });
}

// ============================================================
//  ANIMATION (fixed timestep like working code)
// ============================================================
function animate() {
    requestAnimationFrame(animate);
    var dt = 1 / 60; simTime += dt;
    if (missionStarted && !paused) {
        for (var i = 0; i < Math.ceil(speed * 2); i++) updateSim(dt * 0.5);

        // Tactical pulsing animation for ground rings
        var t = performance.now() / 400; // Speed of pulse
        (missionData.loiters || []).forEach(function (l, i) {
            var groundRing = loiterMeshes[i * 3];
            if (groundRing) {
                // Pulse scale between 0.95 and 1.15
                var scale = 1.05 + 0.10 * Math.sin(t + i);
                groundRing.scale.set(scale, scale, 1);
            }
        });
    }
    if (controls) controls.update();
    updateHUD();
    if (renderer && scene && camera) renderer.render(scene, camera);
}

// ============================================================
//  VIEWER CONTROLS
// ============================================================
function viewerTogglePause() { paused = !paused; var b = document.getElementById('btnPlayPause'); if (b) b.textContent = paused ? 'Play' : 'Pause'; }
function viewerSlower() { speed = Math.max(0.5, speed - 0.5); }
function viewerFaster() { speed = Math.min(10, speed + 0.5); }
function viewerReset() {
    if (!missionData) return;
    missionStarted = false;
    wpIdx = 0; state = 'FLY'; nLoitersDone = 0; distance = 0; battery = 100;
    energyUsedWh = 0; descentWpIdx = 0; currentAltAGL = DRONE_ALT;
    pos = { x: missionData.home.x, y: missionData.home.y };
    heading = 0; trailPoints = []; coveredSet.clear(); paused = false;
    var b = document.getElementById('btnPlayPause'); if (b) b.textContent = 'Pause';
    coverageMeshes.forEach(function (m) { scene.remove(m); if (m.geometry) m.geometry.dispose(); if (m.material) m.material.dispose(); });
    coverageMeshes = [];
    // Rebuild scene without mission elements
    buildScene(missionData);
    computeSafePath();
    updateHUDVisibility();
}

// ============================================================
//  CLICK-TO-RESTRICT ZONES
// ============================================================
function toggleRestrictMode() {
    restrictMode = !restrictMode;
    var btn = document.getElementById('btnRestrict');
    if (btn) {
        btn.textContent = restrictMode ? '\u2705 Marking ON' : '\ud83d\udeab Mark Restricted';
        btn.style.background = restrictMode ? 'rgba(239,68,68,0.3)' : '';
        btn.style.borderColor = restrictMode ? '#ef4444' : '';
    }
    // Change cursor
    if (renderer && renderer.domElement) {
        renderer.domElement.style.cursor = restrictMode ? 'crosshair' : '';
    }
}

function onViewerClick(evt) {
    if (!restrictMode || !raycaster || !mapGroundRef || !scene) return;
    var rect = renderer.domElement.getBoundingClientRect();
    mouseVec.x = ((evt.clientX - rect.left) / rect.width) * 2 - 1;
    mouseVec.y = -((evt.clientY - rect.top) / rect.height) * 2 + 1;
    raycaster.setFromCamera(mouseVec, camera);
    var hits = raycaster.intersectObject(mapGroundRef);
    if (hits.length > 0) {
        var pt = hits[0].point;
        var rad = isLAC ? RESTRICT_RADIUS * 2 : RESTRICT_RADIUS;
        customObstacles.push({ x: pt.x, y: pt.z, radius: rad });
        addCustomObstMeshAt(pt.x, pt.z, rad);
        updateZoneCount();
    }
}

function onViewerRightClick(evt) {
    evt.preventDefault();
    if (!restrictMode || !raycaster || !mapGroundRef || !scene) return;
    var rect = renderer.domElement.getBoundingClientRect();
    mouseVec.x = ((evt.clientX - rect.left) / rect.width) * 2 - 1;
    mouseVec.y = -((evt.clientY - rect.top) / rect.height) * 2 + 1;
    raycaster.setFromCamera(mouseVec, camera);
    var hits = raycaster.intersectObjects(customObstMeshes);
    if (hits.length > 0) {
        // Find which custom obstacle this belongs to and remove it
        var hitMesh = hits[0].object;
        var removeIdx = -1;
        for (var i = 0; i < customObstacles.length; i++) {
            if (customObstacles[i].mesh === hitMesh || customObstacles[i].ring === hitMesh) {
                removeIdx = i;
                break;
            }
        }
        if (removeIdx >= 0) {
            var co = customObstacles[removeIdx];
            scene.remove(co.mesh); scene.remove(co.ring);
            if (co.label) scene.remove(co.label);
            var mi = customObstMeshes.indexOf(co.mesh); if (mi >= 0) customObstMeshes.splice(mi, 1);
            mi = customObstMeshes.indexOf(co.ring); if (mi >= 0) customObstMeshes.splice(mi, 1);
            customObstacles.splice(removeIdx, 1);
            updateZoneCount();
        }
    }
}

function addCustomObstMeshAt(x, z, radius) {
    var baseH = isLAC ? getTerrainHeight(x, z) : 0;
    var h = isLAC ? 50 : 20;
    // Red translucent cylinder
    var cyl = new THREE.Mesh(
        new THREE.CylinderGeometry(radius, radius, h, 32),
        new THREE.MeshStandardMaterial({
            color: 0xff2020,
            transparent: true,
            opacity: 0.25,
            roughness: 0.5,
            metalness: 0.1,
            side: THREE.DoubleSide,
        })
    );
    cyl.position.set(x, baseH + h / 2, z);
    scene.add(cyl); customObstMeshes.push(cyl);
    // Pulsing red danger ring on ground
    var ring = new THREE.Mesh(
        new THREE.RingGeometry(radius - 5, radius + 5, 48),
        new THREE.MeshBasicMaterial({
            color: 0xff2020,
            transparent: true,
            opacity: 0.4,
            side: THREE.DoubleSide,
        })
    );
    ring.rotation.x = -Math.PI / 2;
    ring.position.set(x, baseH + 1, z);
    scene.add(ring); customObstMeshes.push(ring);
    // Label
    var lbl = makeTextSprite('NO-FLY', '#ff4040');
    lbl.position.set(x, baseH + h + 15, z);
    scene.add(lbl); customObstMeshes.push(lbl);
    // Store references on the last obstacle entry
    var co = customObstacles[customObstacles.length - 1];
    if (co) { co.mesh = cyl; co.ring = ring; co.label = lbl; }
}

function clearCustomZones() {
    customObstMeshes.forEach(function (m) { scene.remove(m); });
    customObstMeshes = [];
    customObstacles = [];
    updateZoneCount();
}

async function startMission() {
    var btn = document.getElementById('btnReplan');
    if (btn) { btn.textContent = '⏳ Planning...'; btn.disabled = true; }

    // Disable restrict mode when starting
    if (restrictMode) toggleRestrictMode();

    // Use current mission's map type, or default to LAC
    var mapType = (missionData && missionData.map) ? missionData.map.type : 'lac';
    var coords = (missionData && missionData.coordinates) || {};
    var lat = coords.latitude || 34.1526;
    var lon = coords.longitude || 77.5771;
    var alt = parseFloat((document.getElementById('plannerAlt') || {}).value) || 4000;

    // Collect marked restricted zones
    var zones = customObstacles.map(function (co) {
        return { x: co.x, y: co.y, radius: co.radius };
    });

    try {
        console.log('[SUPARNA] Calling /api/mission/generate with zones:', zones.length);
        var resp = await fetch(API_BASE_URL + '/api/mission/generate', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                map_type: mapType,
                altitude_m: alt,
                latitude: lat,
                longitude: lon,
                custom_obstacles: zones.length > 0 ? zones : null,
            })
        });
        var result = await resp.json();
        if (btn) { btn.disabled = false; }

        if (result.success) {
            console.log('[SUPARNA] Mission generated successfully');
            // Load the newly generated mission.json
            await loadMission();
            // Reset simulation state for fresh start
            wpIdx = 0; state = 'FLY'; nLoitersDone = 0; distance = 0; battery = 100;
            energyUsedWh = 0; descentWpIdx = 0; currentAltAGL = DRONE_ALT;
            simTime = 0; speed = 1.0; paused = false;
            if (missionData) {
                pos = { x: missionData.home.x, y: missionData.home.y };
                heading = 0;
            }
            trailPoints = []; coveredSet.clear();
            // Rebuild scene with fresh drone
            if (missionData) {
                buildScene(missionData);
                computeSafePath();
            }
            // Activate mission
            missionStarted = true;
            // Update UI
            if (btn) {
                btn.textContent = '✈️ FLYING';
                btn.style.background = 'rgba(29,185,84,0.2)';
                btn.style.borderColor = '#1db954';
            }
            updateHUDVisibility();
            setHUD('status', 'FLIGHT');
            setHUD('phase', 'Autonomous coverage in progress');
            console.log('[SUPARNA] Mission started - autonomous flight engaged');
        } else {
            if (btn) { btn.textContent = '🚀 START MISSION'; }
            alert('Mission planning failed: ' + (result.detail || 'Unknown error'));
        }
    } catch (e) {
        if (btn) {
            btn.textContent = '🚀 START MISSION';
            btn.disabled = false;
        }
        console.error('[SUPARNA] Mission generation error:', e);
        alert('Mission planning failed: ' + e.message);
    }
}

function updateHUDVisibility() {
    // Show playback controls after mission started, hide zone setup
    var playback = document.getElementById('hud-playback');
    var zoneSetup = document.getElementById('hud-zone-setup');
    if (playback) playback.style.display = missionStarted ? 'flex' : 'none';
    if (zoneSetup) zoneSetup.style.display = missionStarted ? 'none' : 'flex';
    // Update button state based on mission status
    var btn = document.getElementById('btnReplan');
    if (btn) {
        if (missionStarted) {
            btn.disabled = true;
            btn.textContent = '✈️  FLYING';
            btn.style.background = 'rgba(29,185,84,0.2)';
            btn.style.borderColor = '#1db954';
        } else {
            btn.disabled = false;
            btn.textContent = '🚀 START MISSION';
            btn.style.background = 'rgba(0,212,255,0.15)';
            btn.style.borderColor = 'rgba(0,212,255,0.4)';
        }
    }
    var stEl = document.getElementById('hud-state');
    if (stEl) stEl.textContent = missionStarted ? 'ACTIVE' : 'STANDBY';
}

function updateZoneCount() {
    var el = document.getElementById('zoneCount');
    if (el) el.textContent = customObstacles.length + ' zone' + (customObstacles.length !== 1 ? 's' : '');
}

// ============================================================
//  LOAD MISSION
// ============================================================
async function loadMission() {
    console.log('[SUPARNA] Loading mission...');
    try {
        var resp = await fetch('/mission.json');
        if (!resp.ok) throw new Error('HTTP ' + resp.status);
        missionData = await resp.json();
        console.log('[SUPARNA] Mission loaded:', missionData.map.type, missionData.map.width + 'x' + missionData.map.height);
    } catch (e) {
        console.warn('[SUPARNA] No mission:', e.message);
        setHUD('status', 'NO MISSION');
        setHUD('phase', 'Use Mission Planner to generate');
        updateHUDVisibility();
        return;
    }
    pos = { x: missionData.home.x, y: missionData.home.y };
    if (missionData.energy) energyCapacityWh = missionData.energy.battery_capacity_wh || 370;
    if (missionData.descent && missionData.descent.waypoints) descentWps = missionData.descent.waypoints;
    currentAltAGL = DRONE_ALT;
    computeSafePath();
    console.log('[SUPARNA] Safe waypoints:', safeWaypoints.length);
    try {
        buildScene(missionData);
        console.log('[SUPARNA] Scene built OK');
        setHUD('status', 'READY');
        setHUD('phase', 'Mark zones and click START MISSION');
    } catch (err) {
        console.error('[SUPARNA] Build error:', err);
        setHUD('status', 'BUILD ERROR');
        setHUD('phase', err.message);
    }
    state = 'FLY';
    updateHUDVisibility();
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
        var resp = await fetch(API_BASE_URL + '/api/performance/' + alt);
        if (!resp.ok) throw new Error();
        var d = await resp.json();
        el.innerHTML = '<b>Cruise:</b> ' + d.cruise_speed_ms + ' m/s<br><b>Power:</b> ' + d.power_draw_w + ' W<br><b>Loiter R:</b> ' + d.loiter_radius_m + ' m<br><b>Density:</b> ' + d.air_density + ' kg/m\u00B3 (\u03C3=' + d.density_ratio + ')';
    } catch (e) { el.textContent = 'API unavailable'; }
}
async function generateMission() {
    var btn = document.getElementById('btnGenerate');
    if (btn) { btn.textContent = 'Generating...'; btn.disabled = true; }
    var mapType = document.getElementById('plannerMap').value;
    var alt = parseFloat(document.getElementById('plannerAlt').value);
    var seedEl = document.getElementById('plannerSeed');
    var body = { map_type: mapType, altitude_m: alt };
    if (seedEl && seedEl.value) body.seed = parseInt(seedEl.value);
    try {
        var resp = await fetch(API_BASE_URL + '/api/mission/generate', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
        var result = await resp.json();
        if (btn) { btn.textContent = 'Generate Mission'; btn.disabled = false; }
        if (result.success) {
            var pr = document.getElementById('plannerResult');
            if (pr) pr.style.display = 'block';
            var ps = document.getElementById('plannerStats');
            if (ps) ps.innerHTML = '<b>Loiters:</b> ' + result.loiter_count + '<br><b>Waypoints:</b> ' + result.waypoint_count + '<br><b>Energy:</b> ' + (result.energy.total_energy_wh || 0).toFixed(1) + ' Wh<br><b>Descent:</b> ' + (result.descent.n_loops || 0) + ' loops';
            // Reload viewer
            viewerInitialized = false;
            if (renderer) {
                var c = document.getElementById('viewer-canvas-wrap');
                if (c) c.innerHTML = '';
                renderer = null; scene = null;
            }
        }
    } catch (e) {
        if (btn) { btn.textContent = 'Generate Mission'; btn.disabled = false; }
        alert('Generation failed: ' + e.message);
    }
}

// ============================================================
//  TELEMETRY PAGE
// ============================================================
function updateTelemetryPage() {
    if (!missionData) return;
    var energy = missionData.energy || {};
    var ebt = energy.energy_by_type || {};
    var perf = missionData.performance || {};
    var el = function (id) { return document.getElementById(id); };
    if (el('tel-speed')) el('tel-speed').textContent = perf.cruise_speed_ms || '--';
    if (el('tel-power')) el('tel-power').textContent = perf.power_draw_w || '--';
    if (el('tel-radius')) el('tel-radius').textContent = perf.loiter_radius_m || '--';
    if (el('tel-density')) el('tel-density').textContent = perf.density_ratio || '--';
    drawEnergyDonut(ebt, energy);
    drawPhaseTimeline(energy);
    var descent = missionData.descent || {};
    var ms = document.getElementById('missionSummary');
    if (ms) ms.innerHTML = '<b>Loiters:</b> ' + (missionData.loiters ? missionData.loiters.length : 0) + '<br><b>Total Energy:</b> ' + (energy.total_energy_wh || 0).toFixed(1) + ' / ' + (energy.battery_capacity_wh || 370) + ' Wh<br><b>Remaining:</b> ' + (energy.remaining_pct || 0).toFixed(1) + '%<br><b>Duration:</b> ' + (energy.total_duration_min || 0).toFixed(1) + ' min<br><b>Descent:</b> ' + (descent.n_loops || 0) + ' loops, ' + (descent.energy_wh || 0).toFixed(1) + ' Wh';
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
        ctx.closePath(); ctx.fillStyle = s.color; ctx.fill();
        startAngle += arc;
    });
    ctx.fillStyle = '#f1f5f9'; ctx.font = 'bold 32px Inter, sans-serif';
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText(used.toFixed(0), cx, cy - 6);
    ctx.font = '12px Inter, sans-serif'; ctx.fillStyle = '#64748b';
    ctx.fillText('Wh / ' + total, cx, cy + 14);
    var legend = document.getElementById('energyLegend');
    if (legend) legend.innerHTML = slices.filter(function (s) { return s.value > 0.1 }).map(function (s) { return '<span style="display:inline-flex;align-items:center;gap:5px;margin-right:12px;margin-bottom:4px"><span style="width:10px;height:10px;border-radius:2px;background:' + s.color + ';display:inline-block"></span>' + s.label + ': ' + s.value.toFixed(1) + '</span>'; }).join('');
}
function drawPhaseTimeline(energy) {
    var el = document.getElementById('phaseTimeline');
    if (!el) return;
    var total = energy.total_duration_min || 1;
    var ebt = energy.energy_by_type || {};
    var items = [
        { label: 'Climb', pct: ((ebt.climb || 0) / (energy.total_energy_wh || 1)) * 100, color: '#00d4ff' },
        { label: 'Transit', pct: ((ebt.transit || 0) / (energy.total_energy_wh || 1)) * 100, color: '#10b981' },
        { label: 'Loiter', pct: ((ebt.loiter || 0) / (energy.total_energy_wh || 1)) * 100, color: '#f59e0b' },
        { label: 'RTB', pct: ((ebt.rtb || 0) / (energy.total_energy_wh || 1)) * 100, color: '#a855f7' },
        { label: 'Descent', pct: ((ebt.descent || 0) / (energy.total_energy_wh || 1)) * 100, color: '#ef4444' },
    ];
    var html = '<div style="display:flex;height:28px;border-radius:8px;overflow:hidden;margin-bottom:12px">';
    items.forEach(function (it) { if (it.pct > 0.5) html += '<div style="width:' + it.pct + '%;background:' + it.color + ';display:flex;align-items:center;justify-content:center;font-size:9px;font-weight:700;color:#fff;min-width:24px">' + it.label + '</div>'; });
    html += '</div><div style="display:flex;gap:14px;flex-wrap:wrap">';
    items.forEach(function (it) { var time = (it.pct / 100) * total; html += '<span style="font-size:11px;color:#94a3b8"><span style="display:inline-block;width:8px;height:8px;border-radius:2px;background:' + it.color + ';margin-right:4px"></span>' + it.label + ': ' + time.toFixed(1) + ' min</span>'; });
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
    var W = missionData.map.width, H = missionData.map.height;
    var stats = [
        ['Map Type', (missionData.map || {}).type || '--'], ['Map Size', W + ' x ' + H + ' m'],
        ['Altitude', (missionData.altitude_m || 0) + 'm AMSL'], ['Loiter Zones', (missionData.loiters || []).length],
        ['Obstacles', (missionData.obstacles || []).length], ['Cruise Speed', (perf.cruise_speed_ms || '--') + ' m/s'],
        ['Power Draw', (perf.power_draw_w || '--') + ' W'], ['Loiter Radius', (perf.loiter_radius_m || '--') + ' m'],
        ['Total Energy', (energy.total_energy_wh || 0).toFixed(1) + ' Wh'], ['Battery', (energy.battery_capacity_wh || 370) + ' Wh'],
        ['Duration', (energy.total_duration_min || 0).toFixed(1) + ' min'], ['Descent Loops', descent.n_loops || 0],
    ];
    var tbody = document.getElementById('statsBody');
    if (tbody) tbody.innerHTML = stats.map(function (s) { return '<tr><td>' + s[0] + '</td><td class="font-mono">' + s[1] + '</td></tr>'; }).join('');
    var isaBody = document.getElementById('isaTable');
    if (isaBody) {
        var alts = [0, 1000, 2000, 3000, 4000, 5000], rows = '';
        for (var i = 0; i < alts.length; i++) {
            try { var r = await fetch(API_BASE_URL + '/api/performance/' + alts[i]); if (r.ok) { var d = await r.json(); var hl = (alts[i] === (missionData.altitude_m || 0)) ? ' style="color:var(--accent);font-weight:700"' : ''; rows += '<tr' + hl + '><td class="font-mono">' + alts[i] + 'm</td><td>' + d.cruise_speed_ms + '</td><td>' + d.power_draw_w + '</td><td>' + d.loiter_radius_m + '</td><td>' + d.stall_speed_ms + '</td></tr>'; } } catch (e) { }
        }
        isaBody.innerHTML = rows || '<tr><td colspan="5" style="color:var(--text-muted)">Start server for ISA data</td></tr>';
    }
}

// ============================================================
//  MISSION PLANNER — Location Presets + Leaflet Map
// ============================================================
var LOCATION_PRESETS = [
    { name: 'LAC — Eastern Ladakh', lat: 34.1526, lon: 77.5771, alt: 4000, mapType: 'lac', zoom: 11, desc: 'Pangong Tso to Galwan Valley — active patrolling area along the Line of Actual Control' },
    { name: 'LoC — Kashmir Sector', lat: 34.08, lon: 74.82, alt: 3000, mapType: 'lac', zoom: 11, desc: 'Line of Control near Kupwara — dense forest & mountainous terrain, high infiltration zone' },
    { name: 'Red Corridor — Chhattisgarh', lat: 20.26, lon: 81.60, alt: 500, mapType: 'random', zoom: 10, desc: 'Bastar-Dantewada region — dense jungle canopy, anti-Naxal operations zone' },
    { name: 'Siachen Glacier', lat: 35.42, lon: 77.10, alt: 5500, mapType: 'lac', zoom: 11, desc: 'World\'s highest battlefield — extreme altitude glacier patrol at 5,500m+' },
    { name: 'Arunachal Pradesh — McMahon Line', lat: 27.10, lon: 92.70, alt: 3500, mapType: 'lac', zoom: 10, desc: 'Tawang sector — disputed McMahon Line, dense vegetation & steep terrain' },
];

var leafletMap = null;
var patrolRect = null;

function initPlanner() {
    // Altitude slider
    var slider = document.getElementById('plannerAlt');
    if (slider) {
        slider.addEventListener('input', function () {
            document.getElementById('plannerAltVal').textContent = this.value + 'm';
            updateISAPreview(parseFloat(this.value));
        });
        updateISAPreview(4000);
    }
    // Init Leaflet map
    initLeafletMap();
    // Apply first preset
    onLocationChange();
}

function initLeafletMap() {
    var container = document.getElementById('plannerMap-leaflet');
    if (!container || typeof L === 'undefined') return;

    leafletMap = L.map(container, {
        center: [34.1526, 77.5771],
        zoom: 11,
        zoomControl: true,
        attributionControl: true,
    });

    // Dark satellite-style tiles (CartoDB dark matter)
    L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
        attribution: '&copy; OpenStreetMap &copy; CARTO',
        subdomains: 'abcd',
        maxZoom: 19,
    }).addTo(leafletMap);

    // Patrol zone rectangle (will be set by onLocationChange)
    patrolRect = L.rectangle([[34.12, 77.54], [34.18, 77.62]], {
        color: '#00d4ff',
        weight: 2,
        fillColor: '#00d4ff',
        fillOpacity: 0.08,
        dashArray: '8 4',
    }).addTo(leafletMap);

    // Fix leaflet rendering when tab becomes visible
    setTimeout(function () {
        if (leafletMap) leafletMap.invalidateSize();
    }, 300);
}

function onLocationChange() {
    var sel = document.getElementById('plannerLocation');
    if (!sel) return;
    var val = sel.value;
    var latEl = document.getElementById('plannerLat');
    var lonEl = document.getElementById('plannerLon');
    var altEl = document.getElementById('plannerAlt');
    var mapEl = document.getElementById('plannerMap');
    var labelEl = document.getElementById('coordLabel');

    if (val === 'custom') {
        // Custom mode — just let user type
        if (labelEl) labelEl.textContent = 'Custom Location — enter your coordinates';
        return;
    }

    var preset = LOCATION_PRESETS[parseInt(val)];
    if (!preset) return;

    // Fill in values
    if (latEl) latEl.value = preset.lat.toFixed(4);
    if (lonEl) lonEl.value = preset.lon.toFixed(4);
    if (altEl) { altEl.value = preset.alt; altEl.dispatchEvent(new Event('input')); }
    if (mapEl) mapEl.value = preset.mapType;
    if (labelEl) labelEl.textContent = preset.desc;

    // Update map overlay
    var infoText = document.getElementById('mapInfoText');
    var infoCoords = document.getElementById('mapInfoCoords');
    if (infoText) infoText.textContent = preset.name;
    if (infoCoords) infoCoords.textContent = preset.lat.toFixed(4) + '°N ' + preset.lon.toFixed(4) + '°E | ' + preset.alt + 'm AMSL';

    // Move Leaflet map
    if (leafletMap) {
        leafletMap.flyTo([preset.lat, preset.lon], preset.zoom, { duration: 1.5 });
        // Update patrol rectangle (~4km x 3km box)
        var dLat = 0.018; // ~2km in lat
        var dLon = 0.025; // ~2km in lon
        if (patrolRect) {
            patrolRect.setBounds([
                [preset.lat - dLat, preset.lon - dLon],
                [preset.lat + dLat, preset.lon + dLon]
            ]);
        }
    }
}

async function updateISAPreview(alt) {
    var el = document.getElementById('isaData');
    if (!el) return;
    try {
        var resp = await fetch(API_BASE_URL + '/api/performance/' + alt);
        if (!resp.ok) throw new Error();
        var d = await resp.json();
        el.innerHTML =
            '<b>Cruise:</b> ' + d.cruise_speed_ms + ' m/s &nbsp;|&nbsp; ' +
            '<b>Power:</b> ' + d.power_draw_w + ' W<br>' +
            '<b>Loiter R:</b> ' + d.loiter_radius_m + ' m &nbsp;|&nbsp; ' +
            '<b>\u03C1:</b> ' + d.air_density + ' kg/m\u00B3';
    } catch (e) { el.textContent = 'Start server for ISA data'; }
}

async function generateMission() {
    var btn = document.getElementById('btnGenerate');
    if (btn) { btn.textContent = '\u23F3 Generating...'; btn.disabled = true; }

    var mapType = (document.getElementById('plannerMap') || {}).value || 'lac';
    var alt = parseFloat((document.getElementById('plannerAlt') || {}).value) || 0;
    var lat = parseFloat((document.getElementById('plannerLat') || {}).value) || 34.1526;
    var lon = parseFloat((document.getElementById('plannerLon') || {}).value) || 77.5771;
    var seedEl = document.getElementById('plannerSeed');
    var body = { map_type: mapType, altitude_m: alt, latitude: lat, longitude: lon };
    if (seedEl && seedEl.value) body.seed = parseInt(seedEl.value);

    // Get location name
    var sel = document.getElementById('plannerLocation');
    var locName = (sel && sel.value !== 'custom' && LOCATION_PRESETS[parseInt(sel.value)])
        ? LOCATION_PRESETS[parseInt(sel.value)].name
        : lat.toFixed(4) + '°N, ' + lon.toFixed(4) + '°E';

    try {
        var resp = await fetch(API_BASE_URL + '/api/mission/generate', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(body)
        });
        var result = await resp.json();
        if (btn) { btn.textContent = '\ud83d\ude80 Generate Mission'; btn.disabled = false; }
        if (result.success) {
            var pr = document.getElementById('plannerResult');
            if (pr) pr.style.display = 'block';
            var ps = document.getElementById('plannerStats');
            if (ps) {
                ps.innerHTML =
                    '<b>\ud83d\udccd ' + locName + '</b>' +
                    '<br><b>\ud83c\udfaf Loiters:</b> ' + result.loiter_count +
                    ' &nbsp;|&nbsp; <b>\ud83d\udccc Waypoints:</b> ' + result.waypoint_count +
                    '<br><b>\u26a1 Energy:</b> ' + (result.energy.total_energy_wh || 0).toFixed(1) + ' Wh' +
                    ' &nbsp;|&nbsp; <b>\u23f1\ufe0f</b> ' + (result.energy.total_duration_min || 0).toFixed(1) + ' min' +
                    '<br><b>\ud83d\udd04 Descent:</b> ' + (result.descent.n_loops || 0) + ' loops';
            }
            // Reset viewer to reload with new mission
            viewerInitialized = false;
            animRunning = false;
            if (renderer) {
                var c = document.getElementById('viewer-canvas-wrap');
                if (c) c.innerHTML = '';
                renderer = null; scene = null;
            }
        } else {
            alert('Generation failed: ' + (result.detail || 'Unknown error'));
        }
    } catch (e) {
        if (btn) { btn.textContent = '\ud83d\ude80 Generate Mission'; btn.disabled = false; }
        alert('Generation failed: ' + e.message);
    }
}

// ============================================================
//  INIT
// ============================================================
document.addEventListener('DOMContentLoaded', function () {
    initPlanner();
    setTimeout(function () { initViewer(); }, 100);
});

