from ultralytics import YOLO

# Load a model
model = YOLO("runs/detect/train4/weights/best.pt") 

model.predict(source="yolo_train/Img1.png", save=True)