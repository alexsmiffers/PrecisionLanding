import cv2

# Open the default camera
cam = cv2.VideoCapture(0)

# Ensure the camera opened OK
if not cam.isOpened():
    raise RuntimeError("Cannot open the default camera (index 0). Make sure a camera is connected and available.")

# Get the default frame width and height
frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

# We do not record to disk in this version; just show a temporary live feed

try:
    while True:
        ret, frame = cam.read()

        # Skip processing if frame wasn't read successfully
        if not ret or frame is None:
            # Small pause to avoid busy spin if camera temporarily fails
            cv2.waitKey(10)
            continue

        # Display the captured frame (no recording)
        cv2.imshow('Camera', frame)

        # Press 'q' to exit the loop
        if cv2.waitKey(1) == ord('q'):
            break

# Release the capture object
finally:
    # Release the capture object and clean up windows
    cam.release()
    cv2.destroyAllWindows()