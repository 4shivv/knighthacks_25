from fastapi import FastAPI, UploadFile, File
from fastapi.staticfiles import StaticFiles
import os

app = FastAPI()

# Folder to store uploaded images
UPLOAD_FOLDER = "uploads"
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

# Serve uploaded images statically
app.mount("/images", StaticFiles(directory=UPLOAD_FOLDER), name="images")

@app.post("/upload_image")
async def upload_image(file: UploadFile = File(...)):
    file_path = os.path.join(UPLOAD_FOLDER, file.filename)
    with open(file_path, "wb") as buffer:
        buffer.write(await file.read())
    return {
        "filename": file.filename,
        "url": f"/images/{file.filename}"
    }
