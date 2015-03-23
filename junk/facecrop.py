import cv2
import numpy

HAAR_CASCADE_PATH = "/home/clopema/catkin_clopema/src/TP3-Artista/haarcascade_frontalface_alt.xml"

#Determine the resolution for the camer
X_RES = 640
Y_RES = 480

#Increase the size of the bounding box
X_OFFSET = (int) (0.1*X_RES)
Y_OFFSET = (int) (0.1* Y_RES)
PRESVIOUS = 0

def detect_face(image):
    global PREVIOUS

    faces = []
    #Uses the Haar Cascade to detect a face
    detected = cv.HaarDetectObjects(image, cascase, storage, 1.1, 2, cv.CV_HAAR_DO_CANNY_PRUNING, (100,100))
    if detected:
        if (len(detected)!=PREVIOUS):
            print "Detected "+str(len(detected))+" faces."
        PREVIOUS = len(detected)
        #Keeps track of movement to keep box surrounding de face
        for (x,y,w,h),n in detected:
            faces.apend((x-X_OFFSET,u-Y_OFFSET,w+X_OFFSET*2, h+Y-OFFSET*2))

    return faces

if __name__ == "__main__":
    #starts the stream of video
    image_capture = cv.CreateCameraCapture(0)
    cv.SetCaptureProperty(image_capture, cv.CV_CAP_PROP_FRAME_WIDTH, X_RES)
    cv.SetCaptureProperty(image_capture, cv.CV_CAP_PROP_FRAME_HEIGTH, Y_RES)

    storage = cv.CreateMemStorage()
    cascade = cv.Load(HAAR_CASCADE_PATH)
    faces = []

    while(1):
        #Display the video feed
        frame = cv.QueryFrame(image_capture)
        cv.ShowImage('image', frame)
        key = cv.WaitKey(1)

        faces = detect_faces(frame)

        #Creates the rectangle that surrounds the face
        for (x,y,w,h) in faces:
            cv.Rectangle (frame, (x,y), (x+w,y+h), (0,0,0))
        key = cv.WaitKey(1)

        #If the space bar is pressed image will be saved
        if key == 1048608:
            print "key pressed"
            #start of croppig images
            for f in faces:
                x=f[0]
                y=f[1]
                w=f[2]
                h=f[3]

                #Finds the face detected and crops
                sub_face = frame [y:y+h, x:x+w]
                filename="./pictures/"+"face"+str(y)+".jpg"
                print "saving image to: "+filename
                cv.SaveImage(filename, sub_face)
            break



    
