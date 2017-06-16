import cv2
import pygame
import numpy as np
from random import randint
class drum:
    def __init__(self):
        self.bpx, self.bpy = 0,0
        self.rpx, self.rpy = 0,0
        self.bnx, self.bny = 0,0
        self.rnx, self.rny = 0,0
        self.bip = False
        self.rip= False
        self.bi = False
        self.ri = False
    
    def redd(self):
        hsv=cv2.cvtColor(self.img,cv2.COLOR_BGR2HSV)
        red_lower=np.array([160, 100, 180],np.uint8)
        red_upper=np.array([180, 255, 255],np.uint8)
        mask=cv2.inRange(hsv,red_lower,red_upper)
        kernel = np.ones((5,5),np.uint8)
        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
        ret, thresh = cv2.threshold(closing,127,255,0)
        (_,contour,hierarchy)=cv2.findContours(thresh,1,2)
        self.redcontours=contour
        
    def bluee(self):
        hsv=cv2.cvtColor(self.img,cv2.COLOR_BGR2HSV)
        blue_lower=np.array([110,50,50],np.uint8)
        blue_upper=np.array([130,255,255],np.uint8)
        mask=cv2.inRange(hsv,blue_lower,blue_upper)
        kernel = np.ones((5,5),np.uint8)
        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
        ret, thresh = cv2.threshold(closing,127,255,0)
        (_,contours,hierarchy)=cv2.findContours(thresh,1,2)
        self.bluecontours=contours 

    def bluecont(self):
        try:
            areas = self.bluecontours[0]
            moment = cv2.moments(areas)
            self.bnx = int(moment['m10']/moment['m00'])
            self.bny = int(moment['m01']/moment['m00'])
            radius, color, thickness = 15, (0,0,0), 4
            cv2.circle(self.img,(self.bnx,self.bny),radius,color,thickness)
            xComp = abs(self.bpx-self.bnx)
            yComp = abs(self.bpy-self.bny)
            speed=(xComp**2 + yComp**2)**0.5
            self.bluesound(self.bnx,self.bny,speed)
        except:
        	self.bi=False

    def redcont(self):
        try:
            areas = self.redcontours[0]
            moment = cv2.moments(areas)
            self.rnx = int(moment['m10']/moment['m00'])
            self.rny = int(moment['m01']/moment['m00'])
            radius, color, thickness = 15, (0,0,255), 4
            cv2.circle(self.img,(self.rnx,self.rny),radius,color,thickness)
            xComp = abs(self.rpx-self.rnx)
            yComp = abs(self.rpy-self.rny)
            speed=(xComp**2 + yComp**2)**0.5
            self.redsound(self.rnx,self.rny,speed)
        except:
        	self.ri=False
    def redsound(self,k1,k2,speed):
        #k=(k1,k2)
        if k1>130 and k1<180 and k2>470 and k2<530 and speed>10:
        	self.ri=True
        	if self.ri and not self.rip:
	            pygame.mixer.init()
	            pygame.mixer.music.load("sidedish.wav")
	            pygame.mixer.music.play()
            
            
        if k1>270 and k1<330 and k2>570 and k2<630 and speed>10:
        	self.ri=True
        	if self.ri and not self.rip:
	            pygame.mixer.init()
	            pygame.mixer.music.load("bottomdrum.wav")
	            pygame.mixer.music.play()

        if k1>430 and k1<700 and k2>570 and k2<630 and speed>10:
        	self.ri=True
        	if self.ri and not self.rip:
	            pygame.mixer.init()
	            pygame.mixer.music.load("tomtomdrum6.wav")
	            pygame.mixer.music.play()

        if k1>570 and k1<630 and k2>570 and k2<630 and speed>10:
        	self.ri=True
        	if self.ri and not self.rip:
	            pygame.mixer.init()
	            pygame.mixer.music.load("tomtomdrum7.wav")
	            pygame.mixer.music.play()

        if k1>730 and k1<770 and k2>570 and k2<630 and speed>10:
        	self.ri=True
        	if self.ri and not self.rip:
	            pygame.mixer.init()
	            pygame.mixer.music.load("sound1.mp3")
	            pygame.mixer.music.play()

        if k1>870 and k1<930 and k2>470 and k2<530 and speed>10:
        	self.ri=True
        	if self.ri and not self.rip:
	            pygame.mixer.init()
	            pygame.mixer.music.load("cowbell9.wav")
	            pygame.mixer.music.play()
    def bluesound(self,k1,k2,speed):
        #k=(k1,k2)
        if k1>130 and k1<180 and k2>470 and k2<530 and speed>10:
        	self.si=True
        	if self.si and not self.bip:
	            pygame.mixer.init()
	            pygame.mixer.music.load("sidedish.wav")
	            pygame.mixer.music.play()
            
            
        if k1>270 and k1<330 and k2>570 and k2<630 and speed>10:
        	self.si=True
        	if self.si and not self.bip:
	            pygame.mixer.init()
	            pygame.mixer.music.load("bottomdrum.wav")
	            pygame.mixer.music.play()

        if k1>430 and k1<700 and k2>570 and k2<630 and speed>10:
        	self.si=True
        	if self.si and not self.bip:
	            pygame.mixer.init()
	            pygame.mixer.music.load("tomtomdrum6.wav")
	            pygame.mixer.music.play()

        if k1>570 and k1<630 and k2>570 and k2<630 and speed>10:
        	self.si=True
        	if self.si and not self.bip:
	            pygame.mixer.init()
	            pygame.mixer.music.load("tomtomdrum7.wav")
	            pygame.mixer.music.play()

        if k1>730 and k1<770 and k2>570 and k2<630 and speed>10:
        	self.si=True
        	if self.si and not self.bip:
	            pygame.mixer.init()
	            pygame.mixer.music.load("sound1.mp3")
	            pygame.mixer.music.play()

        if k1>870 and k1<930 and k2>470 and k2<530 and speed>10:
        	self.si=True
        	if self.si and not self.bip:
	            pygame.mixer.init()
	            pygame.mixer.music.load("cowbell9.wav")
	            pygame.mixer.music.play()
	            
    def play(self):
        cap=cv2.VideoCapture(0)
        while(1):
            _,self.img=cap.read()
            self.img = cv2.flip(self.img,1)
            ratio = 1000.0 / self.img.shape[1]
            dim = (1000, int(self.img.shape[0] * ratio))
            self.img= cv2.resize(self.img,dim,interpolation = cv2.INTER_CUBIC)
            self.redd()
            self.bluee()
            self.bluecont()
            self.redcont()
            self.bpx,self.bpy = self.bnx,self.bny
            self.rpx,self.rpy= self.rnx,self.rny
            self.bip= self.bi
            self.rip = self.ri
            b=randint(1,255)
            g=randint(1,255)
            r=randint(1,255)
            
            cv2.circle(self.img,(150,500), 50, (b,g,r), -1)#sidedish
            cv2.circle(self.img,(150,500), 20, (255,0,0), -1)#center
            cv2.circle(self.img,(300,600), 50, (r,g,b), -1)#bootomdrum
            cv2.circle(self.img,(300,600), 20, (255,0,0), -1)#center
            cv2.circle(self.img,(450,600), 50, (g,b,r), -1)#tomd6
            cv2.circle(self.img,(450,600), 20, (255,0,0), -1)#center
            cv2.circle(self.img,(600,600), 50, (b,g,r), -1)#tom7
            cv2.circle(self.img,(600,600), 20, (255,0,0), -1)#center
            cv2.circle(self.img,(750,600), 50, (b,b,r), -1)#sound1
            cv2.circle(self.img,(750,600), 20, (255,0,0), -1)#center
            cv2.circle(self.img,(900,500), 50, (r,r,g), -1)#bell
            cv2.circle(self.img,(900,500), 20, (255,0,0), -1)#center
            cv2.imshow("sgr",self.img)
            if cv2.waitKey(27)&0xFF==ord('q'):
                    cap.release()
                    cv2.destroyAllWindows()
                    break          
obj=drum()
obj.play()
