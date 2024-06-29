import cv2
import numpy as np

class MotionDetector:
    def __init__(self):
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2()
        self.min_area = 1000
        self.max_area = 20000
        self.erosion_iterations = 2
        self.dilation_iterations = 2

    def process_frame(self, frame):
        # Apply background subtraction
        fg_mask = self.bg_subtractor.apply(frame)

        # Apply morphological operations
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
        fg_mask = cv2.erode(fg_mask, None, iterations=self.erosion_iterations)
        fg_mask = cv2.dilate(fg_mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=self.dilation_iterations)

        # Threshold the foreground mask
        _, thresholded = cv2.threshold(fg_mask, 25, 255, cv2.THRESH_BINARY)

        # Find contours
        contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        motion_detected = False

        # Draw bounding boxes for valid contours
        for contour in contours:
            area = cv2.contourArea(contour)
            if self.min_area <= area <= self.max_area:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                motion_detected = True

        if motion_detected:
            cv2.putText(frame, "Motion Detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(thresholded, "Motion Detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        return frame, fg_mask, thresholded

def main():
    cap = cv2.VideoCapture(0)
    motion_detector = MotionDetector()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        processed_frame, fg_mask, thresholded = motion_detector.process_frame(frame)

        # Display the results
        cv2.imshow('Original with Motion', processed_frame)
        cv2.imshow('Foreground Mask', fg_mask)
        cv2.imshow('Thresholded', thresholded)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
