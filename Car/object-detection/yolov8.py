import cv2
from ultralytics import YOLO
from cv2 import getTickCount, getTickFrequency
import serial  # 导入串口通信库
import time

# 初始化串口通信
try:
    ser = serial.Serial('/dev/ttyAMA1', 115200, timeout=1)
    time.sleep(2)  # 等待串口连接稳定
except serial.SerialException as e:
    print(f"无法打开串口：{e}")
    ser = None  # 如果串口打开失败，设置为 None

# 加载 YOLOv8 模型
ncnn_model = YOLO("./best_ncnn_model")

# 获取摄像头内容，参数 0 表示使用默认的摄像头
cap = cv2.VideoCapture(0)

# 获取视频帧的宽度和高度
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'XVID'), 20.0, (frame_width, frame_height))

while cap.isOpened():
    loop_start = getTickCount()
    success, frame = cap.read()  # 读取摄像头的一帧图像

    if success:
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        # 对当前帧进行目标检测
        results = ncnn_model.predict(source=frame)

        # 获取当前帧的检测结果
        result = results[0]
        boxes = result.boxes  # 获取检测到的所有边界框

        # 遍历所有检测到的对象
        for box in boxes:
            # 提取坐标和其他信息
            x1, y1, x2, y2 = box.xyxy[0]  # 左上角和右下角坐标
            conf = box.conf[0]            # 置信度
            cls = int(box.cls[0])         # 类别索引
      
            # 置信度过滤
            if conf < 0.6:
                 continue  # 跳过置信度低的检测结果

            # 将坐标转换为整数
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            # 获取类别名称
            class_name = ncnn_model.names[cls]

            # 在控制台打印检测结果
            print(f"检测到对象：{class_name}，坐标：({x1}, {y1}), ({x2}, {y2})，置信度：{conf:.2f}")

            # 如果检测到的对象是 'xiaoba'，则通过串口发送坐标
            if class_name == 'xiaoba' and ser:
                message = f"xiaoba {x1} {y1} {x2} {y2}\n"
                ser.write(message.encode('utf-8'))
                print(f"已发送串口消息：{message.strip()}")

            # 在图像上绘制边界框和类别名称
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{class_name} {conf:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        # 计算 FPS
        loop_time = getTickCount() - loop_start
        total_time = loop_time / getTickFrequency()
        FPS = 1 / total_time if total_time > 0 else 0

        # 在图像左上角添加 FPS 文本
        fps_text = f"FPS: {FPS:.2f}"
        cv2.putText(frame, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 0, 255), 2)

        # 如果有图形界面，可以使用以下代码显示图像
        # cv2.imshow('Detection', frame)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break

        # 如果没有图形界面，可以将帧保存到视频文件
        out.write(frame)

        # 或者保存当前帧为图像文件
        # cv2.imwrite('detected_frame.jpg', frame)

    else:
        break

cap.release()
if ser:
    ser.close()
out.release()  # 如果使用了 VideoWriter，需要释放
# cv2.destroyAllWindows()
