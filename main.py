import cv2
import matplotlib as plt

cam = cv2.VideoCapture(0)


while 1:
    ret, frame = cam.read()

    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    edges = cv2.Canny(gray_img, 30, 200)
    contours, heirarchies = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    gaussian = cv2.GaussianBlur(frame, (15, 15), 0)
    gaussian_gray = cv2.cvtColor(gaussian, cv2.COLOR_BGR2RGB)


    cv2.drawContours(gaussian, contours, -1, (0, 255, 0), 2)
    cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)
    # cv2.imshow("feed1", edges)
    cv2.imshow("gaussian", gaussian)
    cv2.imshow("regular", frame)

    

    print(f"# of contous: {str(len(contours))}")

    if cv2.waitKey(1) == ord("q"):
        break

cam.release()
cv2.destroyAllWindows()