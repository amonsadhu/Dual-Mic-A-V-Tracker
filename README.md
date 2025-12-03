# Dual-Mic-A-V-Tracker
Real-time audio-visual localization using ESP32-CAM and a dual-mic TDOA array for low-latency sound direction tracking.

# Core DSP Algorithm (MATLAB)
This is the key algorithm used to get the TDOA from the two microphone signals, $\mathbf{x}_1[n]$ and $\mathbf{x}_2[n]$, which you would port to the embedded C code after simulation.
% MATLAB Code for TDOA Estimation (GCC-PHAT)

% 1. Simulate two signals (replace with your actual input data)
Fs = 44100;     % Sampling Frequency (Hz)
delay_samples = 15; % True TDOA: 15 samples
t = 0:1/Fs:0.1;

% Generate a simple noise burst signal
source_signal = chirp(t, 1000, 0.1, 5000); 

% Add noise
noise1 = 0.1 * randn(size(source_signal));
noise2 = 0.1 * randn(size(source_signal));

% Mic 1: Signal + Noise
x1 = source_signal + noise1; 

% Mic 2: Delayed Signal + Noise
x2 = [zeros(1, delay_samples), source_signal(1:end-delay_samples)] + noise2;

% --- GCC-PHAT IMPLEMENTATION ---

% 2. Compute the cross-power spectral density (Psd)
X1 = fft(x1);
X2 = fft(x2);

% Cross-power spectral density (S_12)
S_12 = X1 .* conj(X2);

% 3. Apply the PHAT Weighting
% PHAT: Divide by the magnitude of S_12 to "whiten" the signal (improve noise immunity)
G_phat = 1 ./ abs(S_12); 
G_phat(isinf(G_phat)) = 0; % Handle potential division by zero

% 4. Compute the GCC-PHAT function (Inverse FFT of the weighted Psd)
R_phat = ifft(G_phat .* S_12);

% 5. Find the lag (TDOA) corresponding to the maximum correlation
[max_corr, max_index] = max(abs(R_phat));

% Convert index to time delay (lag) relative to the center of the cross-correlation
N = length(R_phat);
lags = (-N/2 : N/2-1) / Fs; % Time axis for correlation
max_lag_time = lags(max_index); % TDOA in seconds

% Convert TDOA to Angle of Arrival (AOA)
% Assuming speed of sound (c) = 343 m/s and mic spacing (d)
c = 343; % m/s
d = 0.05; % e.g., 5 cm mic spacing (m)

% AOA calculation: TDOA = (d * sin(theta)) / c
% theta = asin((TDOA * c) / d)
theta_rad = asin((max_lag_time * c) / d);
theta_deg = rad2deg(theta_rad);

fprintf('Calculated TDOA: %.6f seconds\n', max_lag_time);
fprintf('Calculated AOA: %.2f degrees\n', theta_deg);

%

# Embedded Pipeline (ESP32 C++)
This code snippet focuses on I2S (Inter-IC Sound) setup, which is the standard way to get high-speed audio data into the ESP32, and then sending the data/result over Serial.

// ESP32-S3/C3 (IDF/Arduino) C++ Snippet for I2S Audio Capture

#include "driver/i2s.h"
#include "esp_log.h"

// Configuration constants (adjust for your specific I2S/Mic setup)
#define I2S_PORT            I2S_NUM_0
#define I2S_BCLK_PIN        GPIO_NUM_32 // Example GPIOs
#define I2S_LRCK_PIN        GPIO_NUM_33
#define I2S_DATA_PIN        GPIO_NUM_34
#define SAMPLE_RATE         44100       // Must match the DSP code
#define I2S_CHANNEL_NUM     2           // Dual mic (stereo mode)
#define BUFFER_SIZE_SAMPLES 1024        // Size of the capture buffer

// Global buffer to hold the raw audio data
int32_t raw_i2s_samples[BUFFER_SIZE_SAMPLES * I2S_CHANNEL_NUM];

void i2s_init() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX), // Master RX
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // Typical for PDM/Delta-Sigma Mics
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // Separate channels
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0, // Default interrupt priority
        .dma_buf_count = 8,
        .dma_buf_len = 64, // DMA buffer size in samples
        .use_apll = false
    };

    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

    i2s_pin_config_t pin_config = {
        .bclk_io_num = I2S_BCLK_PIN,
        .ws_io_num = I2S_LRCK_PIN,
        .data_in_io_num = I2S_DATA_PIN,
        .data_out_io_num = I2S_PIN_NO_CHANGE // RX only
    };

    i2s_set_pin(I2S_PORT, &pin_config);
}

void setup() {
    Serial.begin(115200);
    i2s_init();
    // Initialize ESP32-CAM here (or use the separate CAM example code)
}

void loop() {
    size_t bytes_read = 0;
    
    // Read a block of audio data from I2S
    i2s_read(I2S_PORT, raw_i2s_samples, sizeof(raw_i2s_samples), &bytes_read, portMAX_DELAY);

    if (bytes_read > 0) {
        // Simple Example: Send the calculated Angle of Arrival (AOA)
        // **In a real system, the GCC-PHAT DSP would be implemented here in C/C++**
        
        // --- Placeholder for TDOA/AOA Calculation ---
        float calculated_aoa = 45.0; // Assume AOA was calculated in C++ DSP
        
        // Send a simple, parsable string to the host PC
        Serial.printf("AOA:%f\n", calculated_aoa); 
        // Example output: AOA:45.000000

        // If a new video frame is ready from ESP32-CAM, send its ID/Timestamp too.
    }
}

# Host PC Synchronization (Python/OpenCV)
This Python script runs on the PC, listens to the serial data from the ESP32 (the AOA), reads the live video stream from the ESP32-CAM (via Wi-Fi/RTSP or a separate USB camera), and draws the localization result onto the video feed.
# Python Script for Video Synchronization and Visualization

import cv2
import serial
import time
import re

# --- Configuration ---
# ESP32 Serial Port (Change 'COMX' to your actual port name, e.g., '/dev/ttyUSB0')
SERIAL_PORT = 'COM3' 
BAUD_RATE = 115200
# ESP32-CAM Video Stream URL (RTSP/MJPEG)
VIDEO_STREAM_URL = 'http://192.168.1.10/mjpeg/1' 

# Initialize Serial Communication
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
print(f"Connected to Serial Port: {SERIAL_PORT}")

# Initialize Video Capture (for ESP32-CAM stream)
cap = cv2.VideoCapture(VIDEO_STREAM_URL)
if not cap.isOpened():
    print(f"Error: Could not open video stream at {VIDEO_STREAM_URL}")
    exit()

# Global variable for the last received AOA
current_aoa = 0.0

# Function to draw the AOA direction line on the frame
def draw_aoa(frame, angle_deg):
    (h, w) = frame.shape[:2]
    center = (w // 2, h) # Sound source is usually tracked from the bottom center of the frame
    
    # Normalize angle: 0 deg is straight ahead (up), +/- 90 is left/right
    # Convert to standard Cartesian angle (0 deg = right, 90 deg = up)
    # 0 (straight) -> 90 deg, -90 (left) -> 180 deg, 90 (right) -> 0 deg
    cartesian_angle = 90 - angle_deg
    
    # Calculate the end point of the line
    line_length = h // 4 # Line length proportional to frame height
    angle_rad = (cartesian_angle * cv2.PI) / 180.0
    
    end_x = int(center[0] + line_length * cv2.cos(angle_rad))
    end_y = int(center[1] - line_length * cv2.sin(angle_rad)) # Subtracted because y=0 is top
    
    # Draw the line
    cv2.line(frame, center, (end_x, end_y), (0, 0, 255), 3) # Red line
    # Draw text
    cv2.putText(frame, f"AOA: {angle_deg:.1f} deg", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    
    return frame

# Main loop
while True:
    # 1. Read AOA from Serial Port
    try:
        line = ser.readline().decode('utf-8').strip()
        # Regex to parse the AOA value (e.g., AOA:-45.000000)
        match = re.search(r"AOA:(-?\d+\.\d+)", line)
        if match:
            current_aoa = float(match.group(1))
            print(f"Received AOA: {current_aoa:.2f} deg")
    except Exception as e:
        # print(f"Serial Error: {e}")
        pass

    # 2. Read Video Frame
    ret, frame = cap.read()
    if not ret:
        print("Error reading video frame. Retrying...")
        time.sleep(0.1)
        continue

    # 3. Synchronize and Visualize
    if frame is not None:
        # Overlay the last received AOA onto the current frame
        frame_with_aoa = draw_aoa(frame, current_aoa)
        
        cv2.imshow('Real-Time Audio-Visual Localization', frame_with_aoa)

    # 4. Exit on 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
ser.close()
