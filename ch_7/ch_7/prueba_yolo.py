from ultralytics import YOLO
import cv2

model = YOLO("best.pt")

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()

    if not ret:
        print("No se pudo leer la cámara")
        break

    results = model(frame)

    annotated = results[0].plot()

    cv2.imshow("YOLO Test", annotated)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()
