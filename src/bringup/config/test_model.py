from ultralytics import YOLO

# 1. 加载你刚练好的模型
model = YOLO(r"C:\Users\WIN\Desktop\大唐杯\runs\detect\lego_project3\weights\best.pt") 

# 2. 对一张测试图进行预测（找一张没训练过的积木照片）
results = model.predict(source=r"C:\Users\WIN\Desktop\大唐杯\test photo", save=True, conf=0.5)

# 3. 查看结果
# 运行后，结果会保存在 runs/detect/predict/ 