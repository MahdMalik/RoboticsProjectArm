import time
import cv2

resolution = [1920, 1080]
# between MJPG and YUYV
formatType = "MJPG"

camera = cv2.VideoCapture(0, cv2.CAP_V4L2)

if not camera.isOpened():
    print("Error opening camera!")
else:
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*f"{formatType}"))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
    print(f"width: {camera.get(cv2.CAP_PROP_FRAME_WIDTH)}")
    for i in range(10):
        ret, frame = camera.read()

        if not ret:
            print("didn't get frame")
            break
        
        print(f"frame shape: {frame.shape}")

        cv2.imwrite(f"images/frame{i}.jpg", frame)
        print("saved frame")

    camera.release()
    cv2.destroyAllWindows()
