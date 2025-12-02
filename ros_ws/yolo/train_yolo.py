from ultralytics import YOLO

# Load a model
model = YOLO("yolo11n.pt")  # load a pretrained model (recommended for training)

# Train the model with 2 GPUs
results = model.train(data="yolo_train/MR-Project2.v3i.yolov11/data.yaml", epochs=100, imgsz=640)

# per eseguirlo python3 train_yolo.py