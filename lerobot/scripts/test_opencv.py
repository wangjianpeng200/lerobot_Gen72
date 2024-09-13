import cv2

for ids in range(12):
    try:
        cap = cv2.VideoCapture(ids)

        while cap.isOpened():
            ret, frame = cap.read()
            if ret:
                cv2.imshow(f'Frame {ids}', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                break


        cap.release()
        cv2.destroyAllWindows()

    except:
        continue