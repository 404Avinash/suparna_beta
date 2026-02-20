"""
SUPARNA - FastAPI Mission Control Server
Serves the premium web dashboard and provides REST API for PCCE operations.

Usage:
    python server.py              # Start on port 8000
    python server.py --port 3000  # Custom port
"""

import os
import json
import argparse
from pathlib import Path
from fastapi import FastAPI, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional

# PCCE imports
from src.core.atmosphere import compute_performance, compute_endurance, isa_at_altitude, PERFORMANCE_TABLE
from src.core.geometry import Point

app = FastAPI(title="SUPARNA Mission Control", version="2.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

WEB_DIR = Path("web")
MISSION_FILE = WEB_DIR / "mission.json"
KMZ_FILE = WEB_DIR / "mission.kmz"
REPORT_FILE = WEB_DIR / "mission_report.json"


# === Models ===

class MissionRequest(BaseModel):
    map_type: str = "lac"
    altitude_m: float = 4000.0
    seed: Optional[int] = None


class PerformanceRequest(BaseModel):
    altitude_m: float = 0.0


# === API Routes ===

@app.get("/api/health")
async def health():
    return {"status": "ok", "engine": "SUPARNA PCCE v2.0"}


@app.post("/api/mission/generate")
async def generate_mission(req: MissionRequest):
    """Run the full PCCE pipeline and return mission data."""
    try:
        from export_mission import export_mission
        data = export_mission(
            seed=req.seed,
            map_type=req.map_type,
            altitude_m=req.altitude_m,
        )
        return JSONResponse(content={
            "success": True,
            "stats": data.get("stats", {}),
            "performance": data.get("performance", {}),
            "energy": data.get("energy", {}),
            "descent": {
                "n_loops": data.get("descent", {}).get("n_loops", 0),
                "energy_wh": data.get("descent", {}).get("energy_wh", 0),
            },
            "loiter_count": len(data.get("loiters", [])),
            "waypoint_count": len(data.get("waypoints", [])),
        })
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/mission/latest")
async def get_latest_mission():
    """Return the latest generated mission data."""
    if not MISSION_FILE.exists():
        raise HTTPException(status_code=404, detail="No mission generated yet. Use POST /api/mission/generate first.")
    with open(MISSION_FILE) as f:
        return json.load(f)


@app.get("/api/performance/{altitude_m}")
async def get_performance(altitude_m: float):
    """Get ISA-corrected flight performance at a given altitude."""
    perf = compute_performance(altitude_m)
    atm = isa_at_altitude(altitude_m)
    endurance = compute_endurance(altitude_m)
    return {
        "altitude_m": altitude_m,
        "cruise_speed_ms": perf.cruise_speed_ms,
        "power_draw_w": perf.power_draw_w,
        "loiter_radius_m": perf.loiter_radius_m,
        "stall_speed_ms": perf.stall_speed_ms,
        "descent_rate_m_per_loop": perf.descent_rate_m_per_loop,
        "air_density": round(atm.density, 3),
        "density_ratio": round(atm.density_ratio, 3),
        "temperature_c": round(atm.temperature_celsius, 1),
        "endurance": endurance,
    }


@app.get("/api/performance/table")
async def get_performance_table():
    """Get pre-computed performance at standard altitudes."""
    table = {}
    for alt, perf in PERFORMANCE_TABLE.items():
        table[str(alt)] = {
            "cruise_speed_ms": perf.cruise_speed_ms,
            "power_draw_w": perf.power_draw_w,
            "loiter_radius_m": perf.loiter_radius_m,
            "stall_speed_ms": perf.stall_speed_ms,
        }
    return table


@app.get("/api/export/kmz")
async def download_kmz():
    """Download the generated KMZ file."""
    if not KMZ_FILE.exists():
        raise HTTPException(status_code=404, detail="No KMZ file. Generate a mission first.")
    return FileResponse(KMZ_FILE, media_type="application/vnd.google-earth.kmz", filename="suparna_mission.kmz")


@app.get("/api/export/report")
async def download_report():
    """Download the mission report JSON."""
    if not REPORT_FILE.exists():
        raise HTTPException(status_code=404, detail="No report. Generate a mission first.")
    return FileResponse(REPORT_FILE, media_type="application/json", filename="suparna_report.json")


# === Static Files & SPA ===

app.mount("/css", StaticFiles(directory=str(WEB_DIR / "css")), name="css")
app.mount("/js", StaticFiles(directory=str(WEB_DIR / "js")), name="js")
app.mount("/assets", StaticFiles(directory=str(WEB_DIR / "assets")), name="assets")


@app.get("/mission.json")
async def serve_mission_json():
    if not MISSION_FILE.exists():
        raise HTTPException(status_code=404)
    return FileResponse(MISSION_FILE)


@app.get("/viewer")
async def serve_viewer():
    return FileResponse(WEB_DIR / "index.html")


@app.get("/{path:path}")
async def serve_spa(path: str = ""):
    # Try exact file first
    file_path = WEB_DIR / path
    if file_path.is_file():
        return FileResponse(file_path)
    # Fall back to SPA shell
    spa = WEB_DIR / "app.html"
    if spa.exists():
        return FileResponse(spa)
    return FileResponse(WEB_DIR / "index.html")


if __name__ == "__main__":
    import uvicorn
    parser = argparse.ArgumentParser(description="SUPARNA Mission Control Server")
    parser.add_argument("--port", type=int, default=8000, help="Port (default 8000)")
    parser.add_argument("--host", type=str, default="127.0.0.1")
    args = parser.parse_args()

    # Ensure directories exist
    (WEB_DIR / "css").mkdir(parents=True, exist_ok=True)
    (WEB_DIR / "js").mkdir(parents=True, exist_ok=True)
    (WEB_DIR / "assets").mkdir(parents=True, exist_ok=True)

    print(f"\n  SUPARNA Mission Control")
    print(f"  http://{args.host}:{args.port}\n")
    uvicorn.run(app, host=args.host, port=args.port, log_level="info")
