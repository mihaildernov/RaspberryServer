import sys
sys.path.append("/usr/local/lib/python3.9/site-packages")
import cv2
from dronekit import *
from pymavlink import mavutil
import time
import math
import numpy as np
from pyzbar.pyzbar import decode
from flask import render_template, request, Response, Flask
import RPi.GPIO as GPIO
import time


app = Flask(__name__)

master = mavutil.mavlink_connection('/dev/ttyACM0')
vehicle = connect('/dev/ttyACM0', wait_ready=True)

vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

while not vehicle.armed:
    print("Ждем моторы...")
    time.sleep(1)

@app.route("/")
def index():
    return render_template('robot.html')
    
def gen():
    cap = cv2.VideoCapture(1)
    while True:
        ret, frame = cap.read()
        ret, jpeg = cv2.imencode('.jpg', frame)
        frame=jpeg.tobytes()
        yield (b'--frame\r\n'
        b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        
    vs.release()
    cv2.destroyAllWindows() 

@app.route('/video_feed')
def video_feed():
    return Response(gen(),mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/left_side')
def left_side():
    vehicle.channels.overrides['1'] = 1200
    vehicle.channels.overrides['3'] = 1300
    return 'true'

@app.route('/right_side')
def right_side():
   vehicle.channels.overrides['1'] = 1800
   vehicle.channels.overrides['3'] = 1300
   return 'true'

@app.route('/up_side')
def up_side():
   vehicle.channels.overrides['3'] = 1500
   vehicle.channels.overrides['1'] = 1500
   return 'true'

@app.route('/down_side')
def down_side():
       
   def stop(duration):
       vehicle.channels.overrides['1'] = 1500
       vehicle.channels.overrides['3'] = 1100

   def circle(duration):
       vehicle.channels.overrides['1'] = 1300
       vehicle.channels.overrides['3'] = 1200

   def move_right(duration):
       vehicle.channels.overrides['1'] = 1800
       vehicle.channels.overrides['3'] = 1500

   def move_forward(duration):
       vehicle.channels.overrides['3'] = 1250
       vehicle.channels.overrides['1'] = 1500

   def move_left(duration):
       vehicle.channels.overrides['1'] = 1200
       vehicle.channels.overrides['3'] = 1300
       
   def move_left_fast(duration):
       vehicle.channels.overrides['1'] = 1300
       vehicle.channels.overrides['3'] = 1200
       
   def move_right_fast(duration):
       vehicle.channels.overrides['1'] = 1700
       vehicle.channels.overrides['3'] = 1200
       
   def move_left_slow(duration):
       vehicle.channels.overrides['1'] = 1350
       vehicle.channels.overrides['3'] = 1200
       
   def move_right_slow(duration):
       vehicle.channels.overrides['1'] = 1650
       vehicle.channels.overrides['3'] = 1200

   dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
   parameters = cv2.aruco.DetectorParameters()
   detector = cv2.aruco.ArucoDetector(dictionary, parameters)

   used_codes = []
   
   cap = cv2.VideoCapture(0)
   
   while True:
       ret, image = cap.read()
       image = cv2.resize(image, (640, 480))
       height, width = image.shape[:2]
       top_cutoff = int(height * 0.1)
       bottom_cutoff = int(height * 0.6)
       image = image[top_cutoff:bottom_cutoff, :]
       
       markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(image)
       
       detect = cv2.QRCodeDetector()

       data, bbox, clear_qrcode = detect.detectAndDecode(image)

       minLineLength = 10
       maxLineGap = 20
       
       gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
       blur = cv2.GaussianBlur(gray, (5, 5), 0)
       ret, threshold = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY)
       edges = cv2.Canny(threshold, 50, 150, apertureSize=3)
       lines = cv2.HoughLinesP(edges,1,np.pi/180,10,minLineLength,maxLineGap)
       
       for code in decode(image):
           if code.data.decode('utf-8') not in used_codes:
               print('Продукт найден')
               used_codes.append(code.data.decode('utf-8'))
               
           elif data not in used_codes:
               print('Продукт найден')
               used_codes.append(data)

           elif data in used_codes:
               print('Данный продукт уже был найден ранее')
               
           elif code.data.decode('utf-8'):
               print('Данный продукт уже был найден ранее')

           else:
               pass
       
       print(f"Распознаны следующие продукты: {used_codes}")
       print(f"Количество продуктов: {len(used_codes) - 1}")
       
       if used_codes is not None:
           stop(25)

       if lines is not None:

           for line in lines:
               x1, y1, x2, y2 = line[0]
               
               center_x = (x1 + x2) // 2
               
               if center_x is not None:
                   
                   if center_x >= 500 and markerIds is None:
                       print("Резкий поворот влево")
                       move_right_fast(0.25)

                   elif center_x > 380 and center_x < 500 and markerIds is None:
                       print("Легкий поворот влево")
                       move_right_slow(0.25)

                   elif center_x >= 320 and center_x <= 380 and markerIds is None:
                       print("Продолжение движения")
                       move_forward(0.25)

                   elif center_x > 150 and center_x < 320 and markerIds is None:
                       print("Легкий поворот вправо")
                       move_left_slow(0.25)

                   elif center_x >= 0 and center_x <= 150 and markerIds is None:
                       print("Резкий поворот вправо")
                       move_left_fast(0.25)
                   
                   elif markerIds == 42:
                       print('Поворот влево')
                       stop(0.25)
                       move_left(2.5)
                       
       else:
           print("Линия не найдена")

   return 'true'

if __name__ == "__main__":
 app.run(host='192.168.213.253',port=8081)
