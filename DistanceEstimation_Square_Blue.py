import cv2
import numpy as np
import serial
import time

# configuration serial port
serialcomm = serial.Serial('/dev/ttyUSB0',9600)
serialcomm.timeout = 1

# Tentukan rentang warna biru dalam bentuk HSV
# lower_blue = np.array([90, 100, 100])
# upper_blue = np.array([130, 255, 255])
lower_blue = np.array([145, 120, 130])
upper_blue = np.array([175, 255, 255])

# Baca gambar sebagai input
image = cv2.imread('ReferenceImages/new_200.jpeg')

# Ubah warna dari BGR (Blue, Green, Red) ke HSV (Hue, Saturation, Value)
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Buat mask menggunakan rentang warna biru yang telah ditentukan
mask = cv2.inRange(hsv, lower_blue, upper_blue)

# Temukan kontur dari mask
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

for contour in contours:
    # Hitung luas area kontur
    area = cv2.contourArea(contour)

    # Jika luas area kontur lebih besar dari batas tertentu (misalnya, 100 piksel) untuk menghindari deteksi noise kecil
    if area > 100:
        # Dapatkan bounding box untuk kotak berwarna biru
        x, y, w, h = cv2.boundingRect(contour)

        # Gambar kotak pada gambar asli
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Tampilkan lebar kotak dalam piksel
        width_in_pixels = w
        print("Lebar kotak dalam piksel:", width_in_pixels)
        





# Ukuran objek (dalam sentimeter)
ukuran_objek_cm = 34
jarak_box = 117

# Faktor konversi dari piksel ke sentimeter (disesuaikan dengan ukuran objek dalam citra)
# Anda dapat mengukur beberapa objek dengan ukuran yang diketahui dalam citra untuk mendapatkan faktor konversi ini.
faktor_konversi = 5  # Contoh: 1 cm = 10 piksel

# Fungsi untuk mengukur jarak berdasarkan ukuran objek dan ukuran objek dalam citra
def hitung_jarak(ukuran_objek, ukuran_citra, fokal_panjang):
    return (ukuran_objek * fokal_panjang) / ukuran_citra


def focal_length_finder (measured_distance, real_width, width_in_rf):
    focal_length = (width_in_rf * measured_distance) / real_width
    return focal_length


# finding focal length 
focal_box = focal_length_finder(jarak_box, ukuran_objek_cm, width_in_pixels)

print(f"Focal length: {focal_box}")


# Buka kamera (jika menggunakan video dari file, ganti 0 dengan nama file videonya)
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Konversi ruang warna BGR ke HSV
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Definisikan rentang warna biru dalam HSV
    # lower_blue = np.array([90, 80, 80])
    # upper_blue = np.array([130, 255, 255])
    lower_blue = np.array([145, 120, 130])
    upper_blue = np.array([175, 255, 255])

    # Buat mask dari warna biru
    mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)

    # Operasi morfologi (dilasi dan erosi) untuk membersihkan mask
    kernel = np.ones((5, 5), np.uint8)
    cleaned_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Temukan kontur objek pada mask
    contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        # Filter kontur berdasarkan ukuran objek yang diinginkan
        if cv2.contourArea(contour) > faktor_konversi * ukuran_objek_cm:
            # Gambar persegi di sekitar objek yang dideteksi
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Hitung lebar objek dalam piksel
            lebar_objek_pixel = w

            # Hitung jarak dari kamera terhadap objek
            jarak_cm = hitung_jarak(ukuran_objek_cm, lebar_objek_pixel, focal_box)

            # Tampilkan jarak pada citra
            cv2.putText(frame, f"Jarak: {(jarak_cm*2):.2f} cm", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
               
            # distance parameter
            jarak = int(jarak_cm)*2
            if jarak >155:
                serialcomm.write(b'f') #command forward
            elif jarak >= 135 and jarak <= 155:
                serialcomm.write(b's') #command stop
            elif jarak < 135:
                serialcomm.write(b'b') #command backward
            time.sleep(0.01)
            serialcomm.flushInput()
                
    # Tampilkan citra hasil
    cv2.imshow("Deteksi Objek Biru", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
