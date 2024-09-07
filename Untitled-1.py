# Import necessary libraries
import datetime
import os
import time
import RPi.GPIO as GPIO
from time import sleep
import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
import imageio
import scipy.io
from helper_functions import convert_depth_pixel_to_metric_coordinate
from realsense_device_manager import post_process_depth_frame




# Constants id for pigs
PIG_IDS = [999, 999, 50089, 50221, 50151, 50974, 50919, 50094, 50222, 50981, 
           50043, 50078, 50079, 50031, 50152, 55016, 55017, 55018, 55019, 55020]
# Define total number of pigs
TOTAL_PIGS = 20
# Constants for the pins of the stepper motor and other sensors
STEPPER_PINS = [13, 15, 11, 40, 37, 38]









class Stepper:
    def __init__(self, pins):
        # Initialize the Stepper class with GPIO pin configurations
        self.pins = pins  # Store all pin numbers
        # Unpack pin numbers into specific functions
        self.stepPin, self.directionPin, self.enablePin, self.endPin, self.resetPin, self.magPin = pins
        GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering
        GPIO.setwarnings(False)  # Disable GPIO warnings
        for pin in pins:
            # Set up each pin as either output (for motor control) or input (for sensors)
            GPIO.setup(pin, GPIO.OUT if pin in [self.stepPin, self.directionPin, self.enablePin] else GPIO.IN)
        GPIO.output(self.enablePin, True)  # Initially disable the motor (True means disabled in this setup)

    def cleanGPIO(self):
        # Clean up GPIO settings
        GPIO.cleanup()  # Release all GPIO resources

    def step(self, steps, dir, speed, stayOn=False, docking=False):
        # Control the stepper motor movement
        GPIO.output(self.enablePin, False)  # Enable the motor
        preStatus = GPIO.input(self.magPin)  # Read initial magnetic sensor state
        preResetStatus = GPIO.input(self.resetPin)  # Read initial reset sensor state
        preEndStatus = GPIO.input(self.endPin)  # Read initial end sensor state
        turnLeft = dir == 'right'  # Determine direction (inverted logic here)
        GPIO.output(self.directionPin, turnLeft)  # Set direction pin

        stepCounter = 0  # Initialize step counter
        waitTime = 0.00001 / speed  # Calculate delay between steps
        preMagStatus = [1, 1, 1]  # Initialize magnetic sensor status history
        resetStatus = [1, 1, 1]  # Initialize reset sensor status history
        endStatus = [1, 1, 1]  # Initialize end sensor status history

        while True:
            # Check for various conditions during stepping
            if stepCounter > 2 * steps and not docking:
                if dir == "right":
                    return "right_end"  # Reached right end
                elif dir == "left":
                    return "docked"  # Docking completed
            
            # Perform a single step
            GPIO.output(self.stepPin, True)  # Set step pin high
            sleep(waitTime)  # Wait
            GPIO.output(self.stepPin, False)  # Set step pin low
            
            stepCounter += 1  # Increment step counter
            
            # Update sensor statuses (keep last 3 readings)
            preMagStatus = preMagStatus[1:] + [GPIO.input(self.magPin)]
            resetStatus = resetStatus[1:] + [GPIO.input(self.resetPin)]
            endStatus = endStatus[1:] + [GPIO.input(self.endPin)]
            
            # Check for end conditions
            if sum(endStatus) == 0:
                return "right_end"  # End sensor triggered
            if sum(resetStatus) == 0 and preResetStatus == 1 and docking:
                return "docked"  # Reset sensor triggered during docking
            
            if stepCounter > 1.2 * steps and not docking:
                break  # Exceeded step limit during normal operation
            
            if preStatus == 1 and sum(preMagStatus) == 0 and not docking:
                if stepCounter > 0.3 * steps:
                    return None  # Magnetic sensor triggered after initial movement
            
            if stepCounter > steps or docking:
                if sum(resetStatus) == 0:
                    return "docked"  # Reset sensor triggered after completing steps or during docking

            # Check magnetic sensor
            if self.check_mag_sensor():
                print("Magnetic sensor detected something")
                return "mag_detected"  # Magnetic sensor triggered

        GPIO.output(self.enablePin, True)  # Disable the motor
        return stepCounter  # Return total steps taken if loop completes normally

    def check_mag_sensor(self):
        # Check the status of the magnetic sensor
        mag_status = GPIO.input(self.magPin)
        return mag_status == 0  # Return True if sensor is triggered (assuming 0 means detected)
    
    
def initialize_gpio():
    # Initialize GPIO settings
    GPIO.setmode(GPIO.BOARD)
    for pin in [15, 11, 13]:  # DIR, ENA, STEP
        GPIO.setup(pin, GPIO.OUT)
    GPIO.output(11, GPIO.HIGH)  # Enable motor

def initialize_realsense():
    # Initialize the RealSense camera
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.infrared, 0, 640, 480, rs.format.y8, 30)
    return pipeline, config

def stream_sensor(pig_id, pipeline, config):
    # Capture and process sensor data for a specific pig
    path = f"./ROBOTSOFTWARE/Data/Data_Estrus_2024/PIG{pig_id}/"
    os.makedirs(path, exist_ok=True)

    try:
        pipeline.start(config)
        align_to = rs.stream.depth
        align = rs.align(align_to)
        
        intr = pipeline.get_active_profile().get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        
        f_count = 0
        aligned_frames = None
        
        while f_count < 10:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            
            if depth_frame:
                depth_image = np.asanyarray(depth_frame.get_data())
                depth_image = depth_image[:, 80:560]
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                depth_colormap = cv2.resize(depth_colormap, dsize=(250, 250), interpolation=cv2.INTER_CUBIC)
                
                f_count += 1
        
        if aligned_frames:
            timestamp = datetime.datetime.now()
            filename = f"PID_{pig_id}"
            save_data(aligned_frames, intr, path, filename)
            print(f"Data saved for pig ID: {pig_id}")
        else:
            print(f"No frames captured for pig ID: {pig_id}")
    
    except Exception as e:
        print(f"Error in stream_sensor: {str(e)}")
    finally:
        pipeline.stop()

def save_data(frames, intr, folder_location, fileName):
    # Save captured sensor data to files
    depth_threshold = rs.threshold_filter(min_dist=0.15, max_dist=2.5)
    colorizer = rs.colorizer()        

    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()
    ir_frame = frames.get_infrared_frame()
    
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    ir_image = np.asanyarray(ir_frame.get_data())
    
    filtered_depth_frame = post_process_depth_frame(depth_frame, temporal_smooth_alpha=0.1, temporal_smooth_delta=50)

    XX = np.zeros((480, 480))
    YY = np.zeros((480, 480))
    ZZ = np.zeros((480, 480))
   
    for y in range(480):
        for x in range(480):
            dist = depth_frame.get_distance(x+80, y)
            X, Y, Z = convert_depth_pixel_to_metric_coordinate(dist, x, y, intr)
            XX[y, x] = X
            YY[y, x] = Y
            ZZ[y, x] = Z
    
    obj = np.stack((XX, YY, ZZ))
    matfile = folder_location + "/" + "DM" + fileName + ".mat"
    scipy.io.savemat(matfile, mdict={'out': obj}, oned_as='row')
    imageio.imwrite(folder_location + "/" + "DP" + fileName + ".png", depth_image)
    imageio.imwrite(folder_location + "/" + "IR" + fileName + ".png", ir_image)
    imageio.imwrite(folder_location + "/" + "RGB" + fileName + ".png", color_image)

def handle_stop(sign, stepper):
    # Handle different stop conditions
    if sign == "docked":
        print("docked2")
        action = stepper.step(3000, "left", 5, docking=False)
        handle_stop(action, stepper)
    elif sign == "right_end":
        print("end3")
        action = stepper.step(10000000, "right", 50, docking=True)
        handle_stop(action, stepper)

    # Check magnetic sensor
    mag_detected = check_mag_sensor(stepper)
    if mag_detected:
        print("Magnetic sensor detected something")
        return "mag_detected"

def check_mag_sensor(stepper):
    # Check the status of the magnetic sensor
    mag_status = GPIO.input(stepper.magPin)
    return mag_status == 0  # Assuming 0 means detected, 1 means not detected

def check_end_sensor(stepper):
    # Check the status of the end sensor
    end_status = GPIO.input(stepper.endPin)
    return end_status == 0

def check_reset_sensor(stepper):
    # Check the status of the reset sensor
    reset_status = GPIO.input(stepper.resetPin)
    return reset_status == 0

def main():
    # Initialize GPIO and RealSense camera
    initialize_gpio()
    pipeline, config = initialize_realsense()
    
    # Create a Stepper object to control the motor
    stepper = Stepper(STEPPER_PINS)

    print("Robot starting in 3 seconds...")
    sleep(3)  # Wait for 3 seconds before starting

    pig_index = 0  # Initialize pig index outside the while loop to maintain continuity between cycles

    while True:  # Main loop that runs indefinitely
        print("Starting new cycle")
        for _ in range(TOTAL_PIGS):  # Loop through all pig positions
            if pig_index == 0:  # If at the first position
                print("start")
                action = stepper.step(30000, "left", 0.5, docking=False)  # Move to the first position
            else:  # For subsequent positions
                action = stepper.step(5000, "left", 0.5, docking=False)  # Small step
                action = stepper.step(110000, "left", 0.5, docking=False)  # Larger step
            
            if action == "mag_detected":  # If magnetic sensor is triggered
                print(f"Magnetic sensor triggered at position {pig_index+1}")
                sleep(1)  # Wait for a second before continuing
            elif action == "right_end":  # If end sensor is triggered
                print(f"Ending sensor triggered at position {pig_index}, returning to the reset position")
                pig_index = 0  # Reset pig_index to 0
                break  # Exit the inner loop and continue with the next cycle
            else:
                handle_stop(action, stepper)  # Handle other stop conditions
            
            print(f"moved to {pig_index+1}")

            if PIG_IDS[pig_index] != 999:  # If there's a valid pig ID at this position
                try:
                    stream_sensor(PIG_IDS[pig_index], pipeline, config)  # Capture and save sensor data
                    print(f"Processed pig ID: {PIG_IDS[pig_index]}")
                except Exception as e:
                    print(f"Failed to initialize camera: {e}")
                    sleep(3)
                    pipeline, config = initialize_realsense()  # Reinitialize camera if there's an error
            else:
                sleep(1)  # If no valid pig ID, just wait for a second
            
            pig_index = (pig_index + 1) % TOTAL_PIGS  # Increment pig index and wrap around if necessary

        # After processing all pigs or if ending sensor triggered, move right until reset sensor is triggered
        print("Moving to reset position...")
        while not check_end_sensor(stepper):
            stepper.step(1000, "right", 1000, docking=False)  # Move right until end sensor is triggered
        
        print("End sensor triggered, moving to reset position...")
        while not check_reset_sensor(stepper):
            stepper.step(1000, "right", 500, docking=True)  # Move right until reset sensor is triggered
        
        print("Reset position reached")
        pig_index = 0  # Reset pig_index to 0 after reaching the reset position
        print("Cycle completed. Waiting for 1 minute before starting next cycle...")
        sleep(1)  # Wait for 1 minute before starting the next cycle

if __name__ == "__main__":
    main()  # Run the main function if this script is executed directly