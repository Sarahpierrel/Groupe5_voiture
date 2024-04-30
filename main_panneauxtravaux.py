# -*- coding: utf-8 -*-

#importer ultralytics et torch
from ultralytics import YOLO

# Load a model, we use the nano version
model = YOLO("yolov8n.yaml")  # build a new model from scratch

# Use the model
results = model.train(data="config.yaml", epochs=100)  # train the model

