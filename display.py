import socket
import numpy as np
import cv2

UDP_IP = "192.168.137.189"
UDP_PORT = 12345
IMG_WIDTH = 640
IMG_HEIGHT = 360

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", UDP_PORT))
sock.settimeout(5.0)

def read_image():
    img_bytes = b''
    width, height = IMG_WIDTH, IMG_HEIGHT
    started = False
    ended = False
    while True:
        try:
            data, _ = sock.recvfrom(1500)
        except socket.timeout:
            return None
        line = data.decode('utf-8', errors='ignore').strip()
        if not started:
            parts = line.split(",")
            if line.startswith("START OF IMG"):
                try:
                    width = int(parts[1])
                    height = int(parts[2])
                except (IndexError, ValueError):
                    width = IMG_WIDTH
                    height = IMG_HEIGHT
                print("START OF IMG" + str(width) + "," + str(height))
                started = True
            continue
        if started and not ended:
            if line.startswith("END OF IMG"):
                ended = True
                break
            # If not header/footer, it's image data (may not decode as utf-8)
            if not data.startswith(b"START") and not data.startswith(b"IMG") and not data.startswith(b"END"):
                img_bytes += data
    img_size_bits = width * height
    img_size_bytes = (img_size_bits + 7) // 8
    if len(img_bytes) >= img_size_bytes:
        bits = np.unpackbits(np.frombuffer(img_bytes[:img_size_bytes], dtype=np.uint8))[:img_size_bits]
        img = bits.reshape((height, width)).astype(np.uint8)
        return img
    return None

while True:
    img = read_image()
    if img is not None:
        cv2.imshow('Image', img * 255)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cv2.destroyAllWindows()