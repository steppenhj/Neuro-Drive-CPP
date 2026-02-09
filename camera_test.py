import cv2
import numpy as numpy

def main():
    cap = cv2.VideoCapture(0)

    cap.set(cv2.CAP_DROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_DROP_FRAME_HEIGHT, 240)

    if not cap.isOpened():
        print("error: 카메라 열 수 없음")
        return

    print("카메라 연결 성공 q 누르면 종료")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("frame을 읽을 수 없음")
            break

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        cv2.imshow('Original (RGB)', frame)
        cv2.imshow('Step 1: Grayscale', gray_frame)

        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__=="__main__":
    main()