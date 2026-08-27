"""
main.py
=======
FastAPI Application Entry Point for MapX Navigation & Dead Reckoning Engine.
Serves static frontend files and API/WebSocket routes.
"""

from pathlib import Path
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from backend.routes import router

app = FastAPI(
    title="MapX Navigation Engine",
    description="AI-ML Based Intelligent Dead Reckoning System (ISRO PS 26168)",
    version="2.0.0"
)

# Enable CORS for cross-origin frontend support
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API and WebSocket routes
app.include_router(router)

# Mount static files to serve the frontend web app directly
ROOT_DIR = Path(__file__).resolve().parent.parent
app.mount("/", StaticFiles(directory=str(ROOT_DIR), html=True), name="static")