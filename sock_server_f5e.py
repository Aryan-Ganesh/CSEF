import socket
import struct
import numpy as np 
import cv2
import sys
import argparse
from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput, Log, cudaAllocMapped, cudaResize, cudaToNumpy, cudaFromNumpy
import time
import websocket
import pyttsx3
import time
import subprocess

parser = argparse.ArgumentParser(description="Locate objects in a live camera stream using an object detection DNN.",
                                 formatter_class=argparse.RawTextHelpFormatter,
                                 epilog=detectNet.Usage() + videoSource.Usage() + Log.Usage())

parser.add_argument("input", type=str, default="", nargs='?', help="URI of the input stream")
parser.add_argument("output", type=str, default="", nargs='?', help="URI of the output stream")
parser.add_argument("--network", type=str, default="ssd-mobilenet-v2", help="pre-trained model to load (see below for options)")
parser.add_argument("--overlay", type=str, default="box,labels,conf", help="detection overlay flags (e.g. --overlay=box,labels,conf)\nvalid combinations are:  'box', 'labels', 'conf', 'none'")
parser.add_argument("--threshold", type=float, default=0.5, help="minimum detection threshold to use")
try:
	args = parser.parse_known_args()[0]
except:
	print("")
	parser.print_help()
	sys.exit(0)
input = videoSource(args.input, argv=sys.argv)
#output = videoOutput(args.output, argv=sys.argv)
net = detectNet(args.network, sys.argv, args.threshold)


#text settings for open cv
font = cv2.FONT_HERSHEY_SIMPLEX
thickness = 1
color = (255, 255, 255)
fontScale = 0.5

#-------------------------------------------------------------------
HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 9099         # Port to listen on (non-privileged ports are > 1023)
BUFFER_SIZE = 180*240*4 
#180*240*4 

# socket for haptic belt
ws = websocket.WebSocket()

#Enter IP Address of haptic belt here 
ip = "192.168.1.102:270/ws"
connection = False


# object to generate text to speech 
engine = pyttsx3.init()


def send_msg(msg): #sending data to haptic belt
    if connection:
            return 0
    try:
        ws.send(msg)
        result = ws.recv()
    except OSError as err:
        print(err)
        result = ""
    return result

# processing tof cam data
def process_array(array_size, array_data):
    tmpf=[]
    recv_iter = struct.iter_unpack("=f", array_data)
    for value in recv_iter:
        tmpf.append(value)

    arr = np.array(tmpf)
    arr = arr * (255 / 4)
    arr = np.array(arr, dtype='uint8')
    arr = arr.reshape(180, 240)

    h = 180
    w = 240

    scale = 3.2
    hn = int(h*scale)
    wn = int(w*scale)

    arr = cv2.resize(arr, (wn, hn))
    arr = cv2.rotate(arr, cv2.ROTATE_90_COUNTERCLOCKWISE)

    #over lap zone:-
    x = 180
    start = (2, x)
    end = (576-2, 360 + x)
    #print('size - ', hn, ' x ', wn)

    arr = cv2.rectangle(arr, start, end, (255), 2)
    
    if (cv2.waitKey(1) & 0xFF == ord('q')):
        print("end")
    #print("Type", arr.dtype)
    #print("lenght ", arr.shape)

    # ROI zone - start -    0,   180
    #              end -  540, 640

    return arr

def start_depth_proc():
    subprocess.Popen("/home/phenix/Desktop/Arducam_tof_camera/preview_depth_no_display", shell=True)

def recvall(sock, n):
    # Helper function to recv n bytes or return None if EOF is hit
    data = bytearray()
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data.extend(packet)
    return data

def audio_init():
    engine.setProperty('volume','1.0')
    engine.setProperty('rate',175) 
    #voices = engine.getProperty('voices')
    engine.setProperty('voice', 'english-us')

def main():

    # Text to speech configuration
      #changing index, changes voices. o for male
    audio_init()
    time.sleep(3)
    engine.say('Hello')
    engine.runAndWait()

    try:
        ws.connect("ws://" + ip, timeout=5)
        print("Connected to WebSocket server, IP", ip)
        connection = True
    except OSError as err:
        connection = False
        print(err)

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((HOST, PORT))
        server_socket.listen(1)
        print(f"Server listening on {HOST}:{PORT}")
        
        #start depth
        start_depth_proc()
        # Depth cam
        conn, addr = server_socket.accept()
        print('Connected by', addr)
        tmpb = bytearray(1)
        tmpb[0] = 1
        mean_d = 4
        while True:
            img = input.Capture()   #obj
            if img is None:
                continue

            h = img.height * 0.5
            w = img.width * 0.5
            #print('ir dimn - ', h, 'x', w)
            imgOut = cudaAllocMapped(width=w, height=h, format=img.format)
            cudaResize(img, imgOut)

            np_img = cudaToNumpy(imgOut)
            np_img1 = cv2.rotate(np_img, cv2.ROTATE_180)

            imgOut = cudaFromNumpy(np_img1)
            img = imgOut
            # detect objects in the image (with overlay)
            detections = net.Detect(img, overlay=args.overlay)
                
            # print the detections
            print("detected {:d} objects in image".format(len(detections)))             
                
            #output.Render(img)
            #output.SetStatus("{:s} | Network {:.0f} FPS".format(args.network, net.GetNetworkFPS()));
            
            #net.PrintProfilerTimes()
            if not input.IsStreaming(): #or not output.IsStreaming():
                break

            #DEPTH CAM
            conn.sendall(tmpb)
            data = conn.recv(BUFFER_SIZE, socket.MSG_WAITALL)
            #lines = data.split('\n')
            if data:
                array_size = len(data)
                tof_img = process_array(array_size, data);
                #loop through all detected objects
                for detection in detections:
                        class_name = net.GetClassDesc(detection.ClassID)
                        print(f"Detected '{class_name}'")  
                        print(detection)
                        print(type(detection))
                        print(detection.Top)

                        # bounding zone of object detected
                        
                        t = int(detection.Top)
                        b = int(detection.Bottom)
                        l = int(detection.Left)
                        r = int(detection.Right)

                        # vertical adjustment for tof cam
                        t_adj = t + 180
                        b_adj = b + 180
                        
                        if l > 640:
                                l_adj = 640
                        else:
                                l_adj = l

                        if r > 640:
                                r_adj = 640
                        else:
                                r_adj = r

                        # extracting region of intrest 
                        roi = tof_img[t_adj:b_adj , l_adj:r_adj]
                        tmp_mid = l_adj + (r_adj - l_adj)/2 

                        # calculating distance of the detected object 
                        mean_d = round(roi.mean()/255*4, 2)
                        min_d  = round(roi.min()/255*4, 2)
                        max_d  = round(roi.max()/255*4, 2)                                                      
                        
                        print('roi depth | mean  - ',mean_d, 'm | min - ', min_d, 'm | max - ', max_d, 'm')
                        tof_img = cv2.rectangle(tof_img, (l_adj, t_adj), (b_adj, r_adj), (255), 2)

                        #displaying mean distance value on screen 
                        pos = (l_adj+20, t_adj+20)
                        tof_img = cv2.putText(tof_img, str(mean_d), pos, font,
                                              fontScale, color, thickness, cv2.LINE_AA)
                                                
                        # ROI zone - start -    0,   180
                        #              end -  540,   640
                tof_heatmap = cv2.applyColorMap(tof_img,cv2.COLORMAP_RAINBOW)
                  
                #cv2.imshow("frame_py", tof_heatmap) 

                if(len(detections)==0):
                      instr="00000000"
                      send_msg(instr)
                elif(mean_d < 2.5):
                      # each digit indicates a vibration motor
                      # 1 = on, 0 = off
                      #turning on first and last motors on
                      print(tmp_mid)
                      if(tmp_mid<=270):
                         instr = "00001000"
                      elif tmp_mid>=370:
                         instr = "01000000"
                      else:
                         instr = "00000011"
                       
                      #instr = "00000001"
                      send_msg(instr)
                      engine.say(class_name+'is '+str(int(mean_d)+1)+' feet')
                      engine.runAndWait()
                else:
                      instr = "00000000"
                      send_msg(instr)
        # Close the connection
        print("   -------  CONN closed xxxxxxxx ")
        conn.close()
        ws.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
