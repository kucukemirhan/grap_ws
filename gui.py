import tkinter as tk
import subprocess
import os
import signal
import time

class ROS2LauncherApp:
    def __init__(self, root):
        self.root = root
        self.root.title("ROS2 Launcher")

        # Initialize process variable
        self.process = None

        # Create the TELEOP button
        self.teleop_button = tk.Button(self.root, text="TELEOP", command=self.run_teleop)
        self.teleop_button.pack(pady=20)

        # Create the ARUCO button
        self.aruco_button = tk.Button(self.root, text="ARUCO", command=self.run_aruco)
        self.aruco_button.pack(pady=20)

    def stop_current_process(self):
        """Stop the current running process (if any)."""
        if self.process and self.process.poll() is None:
            print("Stopping the current process...")
            self.process.send_signal(signal.SIGINT)  # Send CTRL+C (SIGINT)
            self.process.wait()  # Wait for the process to finish gracefully
            print("Process stopped.")

    def run_teleop(self):
        """Run the ROS2 launch command for gamepad control."""
        # Stop any currently running process before starting the new one
        self.stop_current_process()

        print("Waiting for command line to be available...")
        time.sleep(2)  # Give it some time to fully stop

        print("Starting ROS2 gamepad launch...")
        self.process = subprocess.Popen(['ros2', 'launch', 'move_program', 'gamepad.launch.py'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    def run_aruco(self):
        """Send CTRL+C to stop the previous command and then run ARUCO launch."""
        # Stop the current process before starting the new one
        self.stop_current_process()

        print("Waiting for command line to be available...")
        time.sleep(2)  # Give it some time to fully stop

        print("Starting ROS2 aruco follow launch...")
        self.process = subprocess.Popen(['ros2', 'launch', 'move_program', 'aruco_follow.launch.py'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

if __name__ == "__main__":
    root = tk.Tk()
    app = ROS2LauncherApp(root)
    root.mainloop()
