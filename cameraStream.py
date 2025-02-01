
import cv2 as cv

cap = cv.VideoCapture('http://192.168.2.168:81/stream')

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
else:
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # If frame is read correctly, ret is True
        if not ret:
            print("Error: Failed to capture image.")
            break

        # Display the resulting frame
        cv.imshow('Camera', frame)

        # Break the loop when 'q' is pressed
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

# Release the capture and close the window
cap.release()
cv.destroyAllWindows()

