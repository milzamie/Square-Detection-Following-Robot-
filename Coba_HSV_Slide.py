import cv2
import numpy as np
# import serial
# import time

# # configuration serial port
# serialcomm = serial.Serial('/dev/ttyUSB0',9600)
# serialcomm.timeout = 1

# Tentukan rentang warna pink dalam bentuk HSV
lower_parameter = np.array([145, 120, 120])
upper_parameter = np.array([175, 255, 255])

# Baca gambar sebagai input
image = cv2.imread('ReferenceImages/New_Data/250.jpg')

# Ubah warna dari BGR (Blue, Green, Red) ke HSV (Hue, Saturation, Value)
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Buat mask menggunakan rentang warna pink yang telah ditentukan
mask = cv2.inRange(hsv, lower_parameter, upper_parameter)
imgResult = cv2.bitwise_and(image,image,mask=mask)

 # Operasi morfologi (dilasi dan erosi) untuk membersihkan mask
kernel = np.ones((5, 5), np.uint8)
cleaned_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

# Temukan kontur dari mask
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

for contour in contours:
    # Hitung luas area kontur
    area = cv2.contourArea(contour)

    # Jika luas area kontur lebih besar dari batas tertentu (misalnya, 100 piksel) untuk menghindari deteksi noise kecil
    if area > 100:
        # Dapatkan bounding box untuk kotak berwarna pink
        x, y, w, h = cv2.boundingRect(contour)

        # Gambar kotak pada gambar asli
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Tampilkan lebar kotak dalam piksel
        width_in_pixels = w
        print("Lebar kotak dalam piksel:", width_in_pixels)
        cv2.imshow("ref image", image)
        cv2.imshow("ref biner", cleaned_mask)


# Ukuran objek (dalam sentimeter)
ukuran_objek_cm = 10
jarak_box = 50

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

focal_box_real = 1500


# Buka kamera (jika menggunakan video dari file, ganti 0 dengan nama file videonya)
cap = cv2.VideoCapture(0)

def empty(a):
    pass

cv2.namedWindow("TrackBars")
cv2.resizeWindow("TrackBars",640,240)
cv2.createTrackbar("Hue Min","TrackBars",145,179,empty)
cv2.createTrackbar("Hue Max","TrackBars",175,179,empty)
cv2.createTrackbar("Sat Min","TrackBars",120,255,empty)
cv2.createTrackbar("Sat Max","TrackBars",255,255,empty)
cv2.createTrackbar("Val Min","TrackBars",120,255,empty)
cv2.createTrackbar("Val Max","TrackBars",255,255,empty)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Konversi ruang warna BGR ke HSV
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h_min = cv2.getTrackbarPos("Hue Min","TrackBars")
    h_max = cv2.getTrackbarPos("Hue Max","TrackBars")
    s_min = cv2.getTrackbarPos("Sat Min","TrackBars")
    s_max = cv2.getTrackbarPos("Sat Max","TrackBars")
    v_min = cv2.getTrackbarPos("Val Min","TrackBars")
    v_max = cv2.getTrackbarPos("Val Max","TrackBars")

    # Definisikan rentang warna dalam HSV
    #pink
    # lower_parameter = np.array([145, 145, 145])
    # upper_parameter = np.array([179, 255, 255])
    #biru
    # lower_parameter = np.array([90, 160, 160])
    # upper_parameter = np.array([130, 255, 255])
    #coklat
    # lower_parameter = np.array([0, 110, 37])
    # upper_parameter = np.array([10, 200, 255])
    #kuning
    # lower_parameter = np.array([22, 80, 150])
    # upper_parameter = np.array([60, 255, 255])
    #hijau
    # lower_parameter = np.array([40, 160, 213])
    # upper_parameter = np.array([80, 255, 255])
    lower_parameter = np.array([h_min, s_min,v_min])
    upper_parameter = np.array([h_max, s_max, v_max])

    # Buat mask dari warna yang ditentukan
    mask = cv2.inRange(hsv_frame, lower_parameter, upper_parameter)
    imgResult = cv2.bitwise_and(frame,frame,mask=mask)

    # Operasi morfologi (dilasi dan erosi) untuk membersihkan mask
    kernel = np.ones((5, 5), np.uint8)
    cleaned_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Temukan kontur objek pada mask
    contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Menghitung jumlah objek yang terdeteksi
    num_objects_detected = len(contours)

    for contour in contours:
        # Filter kontur berdasarkan ukuran objek yang diinginkan
        if cv2.contourArea(contour) > faktor_konversi * ukuran_objek_cm:
            # Gambar persegi di sekitar objek yang dideteksi
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Hitung lebar objek dalam piksel
            lebar_objek_pixel = w

            # Hitung jarak dari kamera terhadap objek
            jarak = hitung_jarak(ukuran_objek_cm, lebar_objek_pixel, focal_box_real)
            jarak_cm = ((jarak*0.3834)-5.5586)*2

            # Tampilkan jarak pada citra
            cv2.putText(frame, f"Jarak: {jarak_cm:.2f} cm", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
               
            # # distance parameter
            # jarak_kirim = int(jarak_cm)
            # if jarak_kirim >77:
            #     serialcomm.write(b'f') #command forward
            # elif jarak_kirim >= 60 and jarak <= 77:
            #     serialcomm.write(b's') #command stop
            # elif jarak_kirim < 60:
            #     serialcomm.write(b'b') #command backward
            # time.sleep(0.01)
            # serialcomm.flushInput()
    

    # # Jika tidak ada objek terdeteksi (tidak ada kontur yang ditemukan)
    # if num_objects_detected == 0:
    #     serialcomm.write(b's')#command stop
    #     time.sleep(0.01)
    #     serialcomm.flushInput()

                
    frame_show = cv2.resize(frame,(640,480))
    # Tampilkan citra hasil
    cv2.imshow("Deteksi Objek", frame_show)

    mask_show = cv2.resize(cleaned_mask,(640,480))
    cv2.imshow("biner", mask_show)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
