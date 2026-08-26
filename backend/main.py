from fastapi import FastAPI
from backend.routes import router

app = FastAPI(
    title="IDR-X Backend",
    description="AI-ML Based Intelligent Dead Reckoning Prototype",
    version="1.0.0"
)


@app.get("/")
def home():
    return {
        "message": "IDR-X Backend is running",
        "status": "success"
    }


app.include_router(router)