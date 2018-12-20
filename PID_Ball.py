import cv2
import numpy as np
import time
import serial
import matplotlib.pyplot as plt

xar = []
yar = []

cap = cv2.VideoCapture(1)

# the center of x axis
position = 240

I = 0

kp = 0.1
ki = -0.025
kd = -0.035  # 0.03
h_actual = 0
pid_i = 0
x = 0
y = 0

calka = 0
pochodna = 0
e_previous = 0

# for i in range(1,21):
#     yar.append(i)

lowerbound = np.array([10, 100, 120])
upperbound = np.array([(25, 255, 255)])

arduinoSerial = serial.Serial('/dev/tty.usbmodem143101', 115200)

time.sleep(2)
print ("ready")
while (True):

    # Capture frame-by-frame
    ret, image = cap.read()

    # size of this image is 1280 x 720 px
    # I change it to 480 x 320 px
    resized_image = cv2.resize(image, (480, 320))

    blurred = cv2.GaussianBlur(resized_image, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, lowerbound, upperbound)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # extract contour
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_NONE)[1]
    # Drawing Contour
    # Processing one contour
    if len(cnts) > 0:
        # compute the center of the  maximum area contour
        m = max(cnts, key=cv2.contourArea)  # finding the contour with maximum area

        M = cv2.moments(m)

        x = int(M["m10"] / M["m00"])
        y = int(M["m01"] / M["m00"])

        # draw the max area contour and center of the shape on the image

        cv2.drawContours(resized_image, [m], -1, (0, 255, 0), 2)

        cv2.circle(resized_image, (x, y), 7, (255, 255, 255), -1)

        # draw red line in bgr color to show the center of screen
        cv2.line(resized_image, (240, 0), (240, 320), (0, 0, 255), 2)

        # draw yellow line in bgr color to show the center of ball
        cv2.line(resized_image, (x, 0), (x, 320), (0, 255, 255), 2)

    # Display the resulting frame
    cv2.imshow('frame', resized_image)
    cv2.imshow('mask', mask)

    # PID calcs

    # error
    e = (position - x)

    # Proportional
    P = kp * e

    # Integral

    # Integral
    # if -20 < e < 20:
    calka += e * ki
    I = ki * calka

    # Derivative

    # calculate elapsed time
    h_previous = h_actual
    h_actual = time.time()
    h = h_actual - h_previous

    pochodna = (e_previous - e) / h
    e_previous = e

    D = kd * pochodna

    radius = 90 + (P + I + D)

    # for me this is bound limited by construction
    if radius <= 60:
        radius = 60
    if radius >= 120:
        radius = 120

    # send radius as a string convert radius to char
    # eg. radius = 120
    # chr(120) = x
    # and convert x (char) to string to send it to arduino
    arduinoSerial.write(str(chr(int(radius))))


    xar.append(e)
    yar.append(h_actual)

    # give ardu same time of delay
    # time.sleep(.050)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
plt.plot(yar, xar)
plt.title("Wykres bledu od czasu dla regulatora PID pilki")
plt.xlabel("Blad")
plt.ylabel("Czas")
plt.show()
cap.release()
cv2.destroyAllWindows()
