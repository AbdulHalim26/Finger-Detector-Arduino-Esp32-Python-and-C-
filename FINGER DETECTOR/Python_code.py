# Buka VScode lalu jalankan setelah kode untuk mikrokontroller-mu sudah terupload, dan
# pastikan sambungan kabel usb tetap terkoneksi ya!

import cv2
import mediapipe as mp
import serial
import time
import numpy as np

class HandDetectionController:
    def __init__(self, com_port='COM4', baud_rate=115200):
        """
        Inisialisasi Hand Detection Controller
        """
        # Setup MediaPipe
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils
        
        # Setup Serial Communication
        try:
            self.serial_conn = serial.Serial(com_port, baud_rate, timeout=1)
            time.sleep(2)  # Wait untuk ESP32 reset
            print(f"Connected to ESP32 on {com_port}")
        except Exception as e:
            print(f"Error connecting to ESP32: {e}")
            self.serial_conn = None
        
        # Setup Camera
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        print("Hand Detection Controller initialized!")
        print("Controls:")
        print("- Show hand to camera to control LEDs")
        print("- Press 'q' to quit")
        print("- Press 's' to toggle serial communication")
    
    def count_fingers(self, landmarks):
        """
        Menghitung jumlah jari yang terbuka menggunakan landmark positions
        Diperbaiki untuk deteksi yang lebih akurat
        """
        # Landmark IDs untuk setiap jari
        # Format: [tip, pip, mcp] untuk setiap jari
        finger_landmarks = [
            [4, 3, 2],    # Thumb: tip, ip, mcp
            [8, 6, 5],    # Index: tip, pip, mcp  
            [12, 10, 9],  # Middle: tip, pip, mcp
            [16, 14, 13], # Ring: tip, pip, mcp
            [20, 18, 17]  # Pinky: tip, pip, mcp
        ]
        
        fingers_up = []
        
        # Deteksi untuk ibu jari (Thumb) - khusus karena orientasinya berbeda
        # Untuk tangan kanan: tip.x > mcp.x menunjukkan jari terbuka
        # Untuk tangan kiri: tip.x < mcp.x menunjukkan jari terbuka
        thumb_tip = landmarks[finger_landmarks[0][0]]
        thumb_mcp = landmarks[finger_landmarks[0][2]]
        
        # Deteksi orientasi tangan berdasarkan posisi wrist (landmark 0) dan middle finger mcp (landmark 9)
        wrist = landmarks[0]
        middle_mcp = landmarks[finger_landmarks[2][2]]
        
        # Jika middle_mcp.x > wrist.x, kemungkinan tangan kanan
        is_right_hand = middle_mcp.x > wrist.x
        
        if is_right_hand:
            # Tangan kanan: thumb terbuka jika tip.x > mcp.x
            thumb_open = thumb_tip.x > thumb_mcp.x
        else:
            # Tangan kiri: thumb terbuka jika tip.x < mcp.x  
            thumb_open = thumb_tip.x < thumb_mcp.x
        
        # Tambahan: periksa juga jarak vertikal untuk memastikan thumb benar-benar terangkat
        thumb_vertical_check = abs(thumb_tip.y - thumb_mcp.y) > 0.02
        fingers_up.append(1 if thumb_open and thumb_vertical_check else 0)
        
        # Deteksi untuk jari lainnya (Index, Middle, Ring, Pinky)
        for i in range(1, 5):
            tip = landmarks[finger_landmarks[i][0]]
            pip = landmarks[finger_landmarks[i][1]]
            mcp = landmarks[finger_landmarks[i][2]]
            
            # Jari terbuka jika:
            # 1. Tip lebih tinggi dari PIP (tip.y < pip.y karena y=0 di atas)
            # 2. PIP lebih tinggi dari MCP (pip.y < mcp.y)
            # 3. Jarak tip-mcp cukup jauh (untuk menghindari false positive)
            finger_straight = (tip.y < pip.y) and (pip.y < mcp.y)
            finger_extended = abs(tip.y - mcp.y) > 0.04  # threshold jarak minimum
            
            fingers_up.append(1 if finger_straight and finger_extended else 0)
        
        return sum(fingers_up)
    
    def send_to_esp32(self, finger_count):
        """
        Mengirim data jumlah jari ke ESP32
        """
        if self.serial_conn and self.serial_conn.is_open:
            try:
                message = f"{finger_count}\n"
                self.serial_conn.write(message.encode())
                return True
            except Exception as e:
                print(f"Error sending data: {e}")
                return False
        return False
    
    def draw_info(self, image, finger_count, fps, finger_states=None):
        """
        Menggambar informasi pada frame
        """
        # Background untuk text
        cv2.rectangle(image, (10, 10), (350, 130), (0, 0, 0), -1)
        
        # Finger count
        cv2.putText(image, f'Jari terdeteksi: {finger_count}', (20, 40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # FPS
        cv2.putText(image, f'FPS: {int(fps)}', (20, 70), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # LED Status
        led_status = "●" * finger_count + "○" * (5 - finger_count)
        cv2.putText(image, f'LED PIN: {led_status}', (20, 100), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # Individual finger status (jika tersedia)
        if finger_states:
            finger_names = ['T', 'I', 'M', 'R', 'P']  # Thumb, Index, Middle, Ring, Pinky
            finger_status = ''.join([f"{name}:{'●' if state else '○'} " 
                                   for name, state in zip(finger_names, finger_states)])
            cv2.putText(image, finger_status, (20, 125), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 1)
    
    def get_finger_states(self, landmarks):
        """
        Mendapatkan status individual setiap jari untuk debugging
        """
        finger_landmarks = [
            [4, 3, 2], [8, 6, 5], [12, 10, 9], [16, 14, 13], [20, 18, 17]
        ]
        
        fingers_up = []
        
        # Thumb
        thumb_tip = landmarks[finger_landmarks[0][0]]
        thumb_mcp = landmarks[finger_landmarks[0][2]]
        wrist = landmarks[0]
        middle_mcp = landmarks[finger_landmarks[2][2]]
        is_right_hand = middle_mcp.x > wrist.x
        
        if is_right_hand:
            thumb_open = thumb_tip.x > thumb_mcp.x
        else:
            thumb_open = thumb_tip.x < thumb_mcp.x
        
        thumb_vertical_check = abs(thumb_tip.y - thumb_mcp.y) > 0.02
        fingers_up.append(thumb_open and thumb_vertical_check)
        
        # Other fingers
        for i in range(1, 5):
            tip = landmarks[finger_landmarks[i][0]]
            pip = landmarks[finger_landmarks[i][1]]
            mcp = landmarks[finger_landmarks[i][2]]
            
            finger_straight = (tip.y < pip.y) and (pip.y < mcp.y)
            finger_extended = abs(tip.y - mcp.y) > 0.04
            
            fingers_up.append(finger_straight and finger_extended)
        
        return fingers_up
    
    def run(self):
        """
        Main loop untuk deteksi tangan
        """
        prev_time = 0
        prev_finger_count = -1
        serial_enabled = True
        
        while True:
            success, image = self.cap.read()
            if not success:
                print("Failed to read from camera")
                break
            
            # Flip image horizontally untuk mirror effect
            image = cv2.flip(image, 1)
            
            # Convert BGR to RGB
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            # Process dengan MediaPipe
            results = self.hands.process(rgb_image)
            
            finger_count = 0
            finger_states = None
            
            # Jika tangan terdeteksi
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    # Gambar landmarks
                    self.mp_draw.draw_landmarks(
                        image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                    
                    # Hitung jumlah jari dan status individual
                    finger_count = self.count_fingers(hand_landmarks.landmark)
                    finger_states = self.get_finger_states(hand_landmarks.landmark)
            
            # Kirim data ke ESP32 hanya jika ada perubahan
            if serial_enabled and finger_count != prev_finger_count:
                if self.send_to_esp32(finger_count):
                    print(f"Sent to ESP32: {finger_count} fingers")
                prev_finger_count = finger_count
            
            # Hitung FPS
            curr_time = time.time()
            fps = 1 / (curr_time - prev_time) if prev_time != 0 else 0
            prev_time = curr_time
            
            # Gambar informasi
            self.draw_info(image, finger_count, fps, finger_states)
            
            # Tampilkan image
            cv2.imshow('Hand Detection - ESP32 LED Controller', image)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                serial_enabled = not serial_enabled
                status = "ENABLED" if serial_enabled else "DISABLED"
                print(f"Serial communication {status}")
        
        self.cleanup()
    
    def cleanup(self):
        """
        Cleanup resources
        """
        # Matikan semua LED sebelum close
        if self.serial_conn and self.serial_conn.is_open:
            self.send_to_esp32(0)
            time.sleep(0.1)
            self.serial_conn.close()
        
        self.cap.release()
        cv2.destroyAllWindows()
        print("Resources cleaned up. Goodbye!")

def main():
    """
    Main function
    """
    print("=== ESP32 Hand Detection LED Controller ===")
    print("Initializing...")
    
    try:
        controller = HandDetectionController(com_port='COM4')
        controller.run()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":

    main()

