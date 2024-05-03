
import sys
import cv2
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QSlider, QTextEdit, QGridLayout, QFrame, QToolButton
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPalette, QColor, QFont,QImage
from PyQt5 import QtGui


class RoverInterface(QMainWindow):
    def __init__(self):
        super().__init__()
        
        self.key_states = {"W": False, "I": False, "A": False, "D": False, "J":False, "K":False, "S": False, "L": False}

        self.setWindowTitle("Rover Control Interface")
        self.setGeometry(100, 100, 1200, 800)

        # Set dark theme palette
        palette = QPalette()
        palette.setColor(QPalette.Window, QColor(53, 53, 53))
        palette.setColor(QPalette.WindowText, QColor(255, 255, 255))
        palette.setColor(QPalette.Base, QColor(25, 25, 25))
        palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
        palette.setColor(QPalette.ToolTipBase, QColor(0, 0, 0))
        palette.setColor(QPalette.ToolTipText, QColor(255, 255, 255))
        palette.setColor(QPalette.Text, QColor(255, 255, 255))
        palette.setColor(QPalette.Button, QColor(53, 53, 53))
        palette.setColor(QPalette.ButtonText, QColor(25, 25, 25))
        palette.setColor(QPalette.BrightText, QColor(255, 0, 0))
        palette.setColor(QPalette.Link, QColor(42, 130, 218))
        palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
        palette.setColor(QPalette.HighlightedText, QColor(0, 0, 0))
        self.setPalette(palette)

        # Create main layout
        main_layout = QHBoxLayout()

        # Create left side layout
        left_layout = QVBoxLayout()

        # Video feed label
        self.video_feed_label = QLabel()
        self.video_feed_label.setAlignment(Qt.AlignCenter)
        self.video_feed_label.setMinimumSize(480, 480)
        self.video_feed_label.setMaximumSize(640, 480)
        left_layout.addWidget(self.video_feed_label)

        # Start the video feed update timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_video_feed)
        self.timer.start(100)  # Update video feed every 100 milliseconds

        # Wheel control system
        wheel_layout = QVBoxLayout()
        self.wheel_status_label = QLabel("Wheel Status: Stable")
        self.wheel_status_label.setFont(QFont('Arial', 12, QFont.Bold))
        self.wheel_status_label.setStyleSheet("background-color: rgb(70, 70, 70); color: white; border: 1px solid gray")
        
        wheel_layout.addWidget(self.wheel_status_label)
        
        wheel_control_layout = QHBoxLayout()  # Change to QHBoxLayout

        

        speed_slider = QSlider(Qt.Vertical)
        wheel_control_layout.addWidget(speed_slider)  # Add slider to the layout first

        wheel_grid = QGridLayout()
        wheel_buttons = [
            ["Left Forward", "Forward", "Right Forward"],
            ["Left", "Stop", "Right"],
            ["Left Backward", "Backward", "Right Backward"]
        ]
        for i in range(3):
            for j in range(3):
                button = QPushButton(wheel_buttons[i][j])
                wheel_grid.addWidget(button, i, j)
                button.clicked.connect(lambda _, cmd=button.text(): self.update_wheel_status(cmd))
        wheel_control_layout.addLayout(wheel_grid)  # Add grid layout after slider
        wheel_layout.addLayout(wheel_control_layout)
        left_layout.addLayout(wheel_layout)

        # Flipper control system
        
        self.flipper_control_layout = QVBoxLayout()
        self.flipper_statuses = {  # Initial flipper statuses
            "Front Left": 0,
            "Front Right": 0,
            "Rear Left": 0,
            "Rear Right": 0
        }
        self.flipper_status = QLabel("Flipper Status: All Flippers Flat")  # Overall status
        self.flipper_status.setFont(QFont('Arial', 12, QFont.Bold))
        self.flipper_status.setStyleSheet("background-color: rgb(70, 70, 70); color: white; border: 1px solid gray")
        self.flipper_control_layout.addWidget(self.flipper_status)

        # Create individual flipper panels
        for flip_name in ["Front Left", "Front Right", "Rear Left", "Rear Right"]:
            flipper_panel = QFrame()
            flipper_panel.setFrameStyle(QFrame.Panel | QFrame.Raised)

            panel_layout = QHBoxLayout()

            status_label = QLabel(f"{flip_name} Status: Flat")
            raise_button = QPushButton("Raise")
            lower_button = QPushButton("Lower")
            raise_button.clicked.connect(lambda _, name=flip_name: self.update_flipper_status(name, 10))
            lower_button.clicked.connect(lambda _, name=flip_name: self.update_flipper_status(name, -10))

            panel_layout.addWidget(status_label)
            panel_layout.addWidget(raise_button)
            panel_layout.addWidget(lower_button)

            flipper_panel.setLayout(panel_layout)
            self.flipper_control_layout.addWidget(flipper_panel)

        print("After creating flipper panels, number of children in flipper_control_layout:", self.flipper_control_layout.count())
        for index in range(1, self.flipper_control_layout.count()):
            panel = self.flipper_control_layout.itemAt(index).widget()
            if isinstance(panel, QFrame):
                print("Panel:", panel)
            
            
        

        emergency_stop = QPushButton("Emergency Stop")
        stabilize = QPushButton("Stabilize")
        emergency_stop.clicked.connect(lambda: self.update_flipper_status("Emergency Stop","esp"))
        stabilize.clicked.connect(lambda: self.update_flipper_status("Stabilize","esp"))
        
        flipper = QHBoxLayout()
        flipper.addWidget(emergency_stop)
        flipper.addWidget(stabilize)
        self.flipper_control_layout.addLayout(flipper)

        left_layout.addLayout(self.flipper_control_layout)

        main_layout.addLayout(left_layout)

        # Create right side layout
        right_layout = QVBoxLayout()

        # Control panel
        control_panel_layout = QGridLayout()  # Use QGridLayout
        mode_status = QLabel("Current Mode: Teleoperation")
        control_panel_layout.addWidget(mode_status, 0, 0, 1, 2)  # Row 0, Column 0 (Spanning 2 columns)

        autonomous_button = QPushButton("Go Autonomous")
        
        control_panel_layout.addWidget(autonomous_button, 1, 0,1,2)  # Row 1, Column 0
        

        object_detection_button = QPushButton("Initiate Object Detection")
        settings_button = QPushButton("Adjust Settings")
        capture_button = QPushButton("Capture Image/Video")
        control_panel_layout.addWidget(object_detection_button, 2, 0)  # Row 2, Column 0
        control_panel_layout.addWidget(settings_button, 2, 1)  # Row 2, Column 1
        control_panel_layout.addWidget(capture_button, 3, 0, 1, 2)  # Row 3, Column 0 (Spanning 2 columns)
        autonomous_button.clicked.connect(lambda: self.log_command("Go Autonomous"))
        object_detection_button.clicked.connect(lambda: self.log_command("Initiate Object Detection"))
        settings_button.clicked.connect(lambda: self.log_command("Adjust Settings"))
        capture_button.clicked.connect(lambda: self.log_command("Capture Image/Video"))

        right_layout.addLayout(control_panel_layout)

        # Log display
        Log_status = QLabel("Log: ")
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)
        right_layout.addWidget(self.log_display)

        main_layout.addLayout(right_layout)

        # Set main layout
        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)
        
    def update_wheel_status(self, command):
        # Example of how to update; you'll need the actual 
        # logic to change wheel direction here
        if command == "Stop":
            self.wheel_status = "Stopped"
        elif command == "Forward":
            self.wheel_status = "Moving Forward"
        elif command == "Left Forward":
            self.wheel_status = "Turning Left Forward"
        elif command == "Right Forward":
            self.wheel_status = "Turning Right Forward"
        elif command == "Left":
            self.wheel_status = "Turning Left"
        elif command == "Right":
            self.wheel_status = "Turning Right"
        elif command == "Left Backward":
            self.wheel_status = "Turning Left Backward"
        elif command == "Backward":
            self.wheel_status = "Moving Backward"
        elif command == "Right Backward":
            self.wheel_status = "Turning Right Backward"

        self.wheel_status_label.setText(f"Wheel Status: {self.wheel_status}")
        self.log_command(command)  # Log the change
        
    def update_flipper_status(self, flipper_name, change):
        if change != "esp":
            current_scale = self.flipper_statuses[flipper_name]
            new_scale = min(max(current_scale + change, -90), 90)  # Ensure the scale stays within -90 to 90 range
            self.flipper_statuses[flipper_name] = new_scale

            if new_scale == 0:
                status = "Flat"
            elif new_scale == 90:
                status = "Raised"
            elif new_scale == -90:
                status = "Standing"
            elif new_scale > 0:
                status = "Partially Raised"
            else:
                status = "Partially Standing"

            # Iterate through flipper panels and update status labels
            for index in range(1, self.flipper_control_layout.count()):
                panel = self.flipper_control_layout.itemAt(index).widget()
                if isinstance(panel, QFrame):
                    panel_layout = panel.layout()
                    status_label = panel_layout.itemAt(0).widget()
                    if status_label.text().startswith(f"{flipper_name} Status:"):
                        status_label.setText(f"{flipper_name} Status: {status}")

            self.log_command(f"{flipper_name} {'Lowered' if change < 0 else 'Raised'}")
        else:
            if flipper_name ==  "Emergency Stop":
                self.flipper_status.setText("Emergency Stop")
            else:
                for index in range(1, self.flipper_control_layout.count()):
                    panel = self.flipper_control_layout.itemAt(index).widget()
                    if isinstance(panel, QFrame):
                        panel_layout = panel.layout()
                        status_label = panel_layout.itemAt(0).widget()
                        
                        status_label.setText(f"{flipper_name} Status: ")        

        # self.log_command(f"{flipper_name} {new_status}")
        
    def log_command(self, command):
        self.log_display.append(f"Command: {command}")
        
    def keyPressEvent(self, event):
        
        if event.key() == Qt.Key_W:
            self.key_states["W"] = True
        elif event.key() == Qt.Key_S:
            self.key_states["S"] = True
        elif event.key() == Qt.Key_I:
            self.key_states["I"] = True
        elif event.key() == Qt.Key_K:
            self.key_states["K"] = True
        elif event.key() == Qt.Key_A:
            self.key_states["A"] = True
        elif event.key() == Qt.Key_D:
            self.key_states["D"] = True
        elif event.key() == Qt.Key_J:
            self.key_states["J"] = True
        elif event.key() == Qt.Key_L:
            self.key_states["L"] = True

        # Check if both "W" and "I" keys are pressed
        if self.key_states["W"] and self.key_states["I"]:
            # Perform the action for both keys being pressed simultaneously
            self.key_states = {"W": False, "I": False, "A": False, "D": False, "J":False, "K":False, "S": False, "L": False}

            self.update_flipper_status("Front Left", 10)
            self.update_flipper_status("Front Right", 10)
            
            self.flipper_status.setText("Flipper Status: Front Flippers Going Up")
        elif self.key_states["A"] and self.key_states["J"]:
            # Perform the action for both keys being pressed simultaneously
            self.key_states = {"W": False, "I": False, "A": False, "D": False, "J":False, "K":False, "S": False, "L": False}

            self.update_flipper_status("Rear Left", 10)
            self.update_flipper_status("Rear Right", 10)
            self.flipper_status.setText("Flipper Status: Back Flippers Going Up")
        elif self.key_states["S"] and self.key_states["K"]:
            # Perform the action for both keys being pressed simultaneously
            self.key_states = {"W": False, "I": False, "A": False, "D": False, "J":False, "K":False, "S": False, "L": False}

            self.update_flipper_status("Front Left", -10)
            self.update_flipper_status("Front Right", -10)
            self.flipper_status.setText("Flipper Status: Front Flippers Going Down")
        elif self.key_states["D"] and self.key_states["L"]:
            # Perform the action for both keys being pressed simultaneously
            self.key_states = {"W": False, "I": False, "A": False, "D": False, "J":False, "K":False, "S": False, "L": False}

            self.update_flipper_status("Rear Left", -10)
            self.update_flipper_status("Rear Right", -10)
            self.flipper_status.setText("Flipper Status: Rear Flippers Going Down")
            
        elif self.key_states["W"]:
            self.update_flipper_status("Front Left", 10)
            self.flipper_status.setText("Flipper Status: Front Left Going Up")
            
        elif self.key_states["I"]:
            self.update_flipper_status("Front Right", 10)
            self.flipper_status.setText("Flipper Status: Front Right Going Up")
        
        elif self.key_states["S"]:
            self.update_flipper_status("Front Left", -10)
            self.flipper_status.setText("Flipper Status: Front Left Going Down")
        elif self.key_states["K"]:
            self.update_flipper_status("Front Right", -10)
            self.flipper_status.setText("Flipper Status: Front Right Going Down")
        
        elif self.key_states["A"]:
            self.update_flipper_status("Rear Left", 10)
            self.flipper_status.setText("Flipper Status: Rear Left Going Up")
        elif self.key_states["J"]:
            self.update_flipper_status("Rear Right",10)
            self.flipper_status.setText("Flipper Status: Rear Right Going Up")
        elif self.key_states["D"]:
            self.update_flipper_status("Rear Left", -10)
            self.flipper_status.setText("Flipper Status: Rear Left Going Down")
            
        elif self.key_states["L"]:
            self.update_flipper_status("Rear Right",-10)
            self.flipper_status.setText("Flipper Status: Rear Right Going Down")
            

        else:
            super().keyPressEvent(event)
            
    def update_video_feed(self):
        # Read a frame from the camera (change the index if you have multiple cameras)
            cap = cv2.VideoCapture(1)
            ret, frame = cap.read()
            
            # Convert the frame from BGR to RGB
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Convert the frame to QImage
            img = QImage(frame_rgb, frame_rgb.shape[1], frame_rgb.shape[0], QImage.Format_RGB888)
            
            # Set the QImage as the pixmap of the video feed label
            self.video_feed_label.setPixmap(QtGui.QPixmap.fromImage(img))
            
            # Release the camera capture
            cap.release()



if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RoverInterface()
    # with open("styles.qss", "r") as file:
    #     app.setStyleSheet(file.read())
    window.show()
    sys.exit(app.exec_())