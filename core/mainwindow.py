# -*- coding: utf-8 -*-

__author__ = "Yu-Sheng Lin"
__copyright__ = "Copyright (C) 2016-2022"
__license__ = "AGPL"
__email__ = "pyquino@gmail.com"

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtGui import QIcon
from PyQt5.uic import loadUi
from PyQt5.QtCore import pyqtSlot

import platform
import numpy as np
import math
import time
import cv2

# check for install the ids camera sdk
try:
    from pyueye import ueye
except ImportError:
    print("Do not install pyueye")
    print("Please use 'pip install pyueye' to install modules")


class Thread_count(QThread):
    test_signal = pyqtSignal(QPixmap)

    def __init__(self, hcam, width, height, sInfo, m_vpcSeqImgMem, bufeersize, nBitsPerPixel, parent=None):
        QThread.__init__(self, parent)
        self.hCam = hcam
        self.width = width
        self.height = height
        self.sInfo = sInfo
        self.m_vpcSeqImgMem = m_vpcSeqImgMem
        self.bufeersize = bufeersize
        self.nBitsPerPixel = nBitsPerPixel

    def run(self):
        width = self.sInfo.nMaxWidth
        height = self.sInfo.nMaxHeight
        # create an event of camera
        ueye.is_EnableEvent(self.hCam, ueye.IS_SET_EVENT_FRAME)
        # ueye.is_FreezeVideo(hCam, ueye.IS_DONT_WAIT)
        count = 0
        # count the frame ret
        if platform.system() == "Windows":
            try:
                import win32event
                import ctypes
                hEvent = win32event.CreateEvent(None, False, False, None)
                hEvent_FL = int(hEvent)
                pEvent_FL = ueye.c_void_p(hEvent_FL)
                # hEvent = ueye.c_void_p()
                ueye.is_InitEvent(self.hCam, pEvent_FL, ueye.IS_SET_EVENT_FRAME)
            except ImportError: pass
        while True:
            iImageID = ueye.c_int(0)
            pBuffer = ueye.c_mem_p()

            if platform.system() == "Linux":
                nRet = ueye.is_WaitEvent(self.hCam, ueye.IS_SET_EVENT_FRAME, 1000)
            elif platform.system() == "Windows":
                nRet = None

                dwRet = ctypes.windll.kernel32.WaitForSingleObject(pEvent_FL, 1000)

                # TODO: need to check how to use event
            if nRet == ueye.IS_SUCCESS or dwRet == win32event.WAIT_OBJECT_0:

                # TODO get data transform self.pcImageMemory
                self.array = ueye.get_data(self.m_vpcSeqImgMem[count - 1], self.width, self.height,
                                      32, 16416, copy=False)
                frame = np.reshape(self.array, (self.height.value, self.width.value, 4))[:, :, 0:3]
                # ...resize the image by a half
                print(frame)
                qImg = QImage(frame, self.width.value, self.height.value, QImage.Format_RGB32) # Format_Grayscale8
                # qImg = qImg.scaled(int(self.width.value / 2), int(self.height.value / 2))
                qpxmp = QPixmap.fromImage(qImg)
                # print(qpxmp)
                self.test_signal.emit(qpxmp)
                ueye.is_UnlockSeqBuf(self.hCam, iImageID, pBuffer)
                count = count + 1

                if platform.system() == "Windows":
                    if dwRet == win32event.WAIT_TIMEOUT:  # win32event.WAIT_FAILED
                        print("timeout in windows")
                        break
                elif platform.system() == "Linux":
                    if nRet != ueye.IS_SUCCESS:
                        print(f"error code {nRet}")
                        break

            if count >= self.bufeersize:
                print(f"this is test {len(self.m_vpcSeqImgMem)}")
                ueye.is_FreezeVideo(self.hCam, ueye.IS_WAIT)
                break
            # i = i + 1
        ueye.is_DisableEvent(self.hCam, ueye.IS_SET_EVENT_FRAME)
        if platform.system() == "Windows" and pEvent_FL != None:
            ueye.is_ExitEvent(self.hCam, ueye.IS_SET_EVENT_FRAME)
            # import win32api
            # win32api.CloseHandle(hEvent)
            ctypes.windll.Kernel32.CloseHandle(pEvent_FL)

class MainWindow(QMainWindow):

    signal1 = pyqtSignal()

    def __init__(self, parent = None):
        super(MainWindow, self).__init__()
        loadUi("core/mainwindow.ui", self)
        # self.__init_setting()
        # self._add_action()
        self.show()

        self._init_ids_camera(0)
        nRet = ueye.is_InitCamera(self.hCam, None)
        if nRet == 3:
            print("Do not find the camera")
            print("Please insert the cable to the computer")
            exit()
        # frame
        self.liveframe = None
        self.bufeersize = None
        self.m_viSeqMemID = []
        self.m_vpcSeqImgMem = []
        self.signal1.connect(self.update_image)

    def _init_ids_camera(self, camera_ids):
        self.hCam = ueye.HIDS(camera_ids)  # 0: first available camera;  1-254: The camera with the specified camera ID
        self.sInfo = ueye.SENSORINFO()
        self.cInfo = ueye.CAMINFO()
        self.pcImageMemory = ueye.c_mem_p()
        self.MemID = ueye.int()
        self.rectAOI = ueye.IS_RECT()
        self.pitch = ueye.INT()
        self.nBitsPerPixel = ueye.INT(24)  # 24: bits per pixel for color mode; take 8 bits per pixel for monochrome
        channels = 3  # 3: channels for color mode(RGB); take 1 channel for monochrome
        self.m_nColorMode = ueye.INT(32)  # Y8/RGB16/RGB24/REG32
        self.bytes_per_pixel = int(self.nBitsPerPixel / 8)
        self.m_buffer_init = 0
        self.fps_second = ueye.DOUBLE()
        self.exposure_time_max = ueye.DOUBLE()
        self.exposure_time_min = ueye.DOUBLE()
        self.gainFactor = ueye.DOUBLE()
        self.maxFrame = ueye.DOUBLE()
        self.miniFrame = ueye.DOUBLE()
        self.intervallFrame = ueye.DOUBLE()

    def camSeqBuild(self):
        bRet = False
        FrameTimeMin = ueye.c_double()
        FrameTimeMax = ueye.c_double()
        FrameTimeIntervall = ueye.c_double()
        nRet = ueye.is_GetFrameTimeRange(self.hCam, FrameTimeMin, FrameTimeMax, FrameTimeIntervall)
        # nRet = 0
        if nRet == ueye.IS_SUCCESS:
            # print(FrameTimeMin)
            maxBuffer = (1 / FrameTimeMin) + 0.5
            # print(maxBuffer)
            if maxBuffer < 3:
                nmaxbuffer = 3
            else:
                nmaxbuffer = self.bufeersize

        imageSize = ueye.IS_RECT()
        nRet = ueye.is_AOI(self.hCam, ueye.IS_AOI_IMAGE_GET_AOI, imageSize, ueye.sizeof(imageSize))
        nAllocSizeX = imageSize.s32Width
        nAllocSizeY = imageSize.s32Height
        self.width = imageSize.s32Width
        self.height = imageSize.s32Height

        nAbsPosX = ueye.c_void_p()
        nAbsPosY = ueye.c_void_p()
        ueye.is_AOI(self.hCam, ueye.IS_AOI_IMAGE_GET_POS_X_ABS, nAbsPosX, ueye.sizeof(nAbsPosX))
        ueye.is_AOI(self.hCam, ueye.IS_AOI_IMAGE_GET_POS_X_ABS, nAbsPosY, ueye.sizeof(nAbsPosY))

        if nAbsPosX:
            nAllocSizeX = self.sInfo.nMaxWidth
        if nAbsPosY:
            nAllocSizeY = self.sInfo.nMaxHeight

        # allocate buffer (memory) in a for-loop
        for i in range(0, nmaxbuffer):
            iImageID = ueye.c_int(0)
            pcImhMem = ueye.c_mem_p()
            nRet = ueye.is_AllocImageMem(self.hCam, nAllocSizeX, nAllocSizeY, self.nBitsPerPixel, pcImhMem, iImageID)
            if nRet != ueye.IS_SUCCESS:
                break
            nRet = ueye.is_AddToSequence(self.hCam, pcImhMem, iImageID)
            if nRet != ueye.IS_SUCCESS:
                ueye.is_FreeImageMem(self.hCam, pcImhMem, iImageID)
                break

            self.m_viSeqMemID.append(iImageID)
            self.m_vpcSeqImgMem.append(pcImhMem)

        # nRet = ueye.is_InitImageQueue(hCam, 0)
        if nRet == ueye.IS_SUCCESS:
            bRet = True
        else:
            bRet = False
        return bRet

    @pyqtSlot()
    def on_testbtn_clicked(self):
        print(123)
        self.all_process()
        # load the config from yaml
        # expos_time = 12.46
        # clock_time = 474
        # capcture_img = 300
        # use_automodel = 80
        # open_ai_classifier = False
        # save_path = ""
        # detection_folder = ""
        # fps = ""
        # test_use = ""
        #
        #
        # # setting all camera config from yaml
        # gain_value = int(0)
        # clock_time0_p = ueye.INT(0)
        # clock_time1 = ueye.int(clock_time)  # clock time set
        #
        # exposure_value = ueye.double(expos_time)  # uints ns
        # # ueye.is_SetHWGainFactor(self.hCam, ueye.IS_SET_MASTER_GAIN_FACTOR, gain_value)
        #
        # # self.test_thread = Thread_count(self.hCam, self.width, self.height, self.sInfo, self.m_vpcSeqImgMem,
        # #                                 self.bufeersize, self.nBitsPerPixel)

    def all_process(self):
        """This function connect all process"""
        # Build Camera Sequence
        self.bufeersize = 10 # self.number_of_shots.value()
        bRet = self.camSeqBuild()

        if bRet is True:
            print("IS_SUCCESS")
        else:
            self.CamSeqKill()
            ueye.is_ExitCamera(self.hCam)
            print("Error")

        # build thread
        # t = threading.Thread(target=self.job)

        ueye.is_CaptureVideo(self.hCam, ueye.IS_DONT_WAIT)

        self.test_thread = Thread_count(self.hCam, self.width, self.height, self.sInfo, self.m_vpcSeqImgMem,
                                        self.bufeersize, self.nBitsPerPixel)
        self.test_thread.test_signal.connect(self.update_image)
        self.test_thread.finished.connect(self.savve)
        self.test_thread.start()

    def savve(self):
        print(len(self.m_vpcSeqImgMem))
        self.CamSeqKill()

    def CamSeqKill(self):
        # ueye.is_ExitImageQueue(hCam)
        ueye.is_ClearSequence(self.hCam)
        # Free buffer
        for i in range(0, len(self.m_viSeqMemID)):
            Ret = ueye.is_FreeImageMem(self.hCam, self.m_vpcSeqImgMem[i], self.m_viSeqMemID[i])
        self.m_vpcSeqImgMem.clear()
        self.m_viSeqMemID.clear()

    def update_image(self, data):
        self.image1.setPixmap(data)
        self.image1.setScaledContents(True)
