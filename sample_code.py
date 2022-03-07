from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtGui import QIcon
from PyQt5.uic import loadUi
from PyQt5.QtCore import pyqtSlot
from ids_peak import ids_peak

import platform
import numpy as np
import math
import time
import cv2
import os
import datetime

# check for install the ids camera sdk
try:
    from pyueye import ueye
except ImportError:
    print("Do not install pyueye")
    print("Please use 'pip install pyueye' to install modules")


class Thread_show_image(QThread):
    send_image = pyqtSignal(QPixmap)
    """這邊主要是主畫面顯示使用，live mode 開啟相機"""

    def __init__(self, hcam, width, height, sInfo, nBitsPerPixel, parent=None):
        QThread.__init__(self, parent)
        self.hCam = hcam
        self.width = width
        self.height = height
        self.sInfo = sInfo
        self.nBitsPerPixel = nBitsPerPixel
        self.pcImageMemory = ueye.c_mem_p()
        self.MemID = ueye.int()
        self.pitch = ueye.INT()
        self.stop_thread = False

        rectAOI = ueye.IS_RECT()
        nRet = ueye.is_AOI(self.hCam, ueye.IS_AOI_IMAGE_GET_AOI, rectAOI, ueye.sizeof(rectAOI))
        if nRet != ueye.IS_SUCCESS:
            print("is_AOI ERROR")
            exit()

        self.width = rectAOI.s32Width
        self.height = rectAOI.s32Height

        # ueye.is_SetColorMode(self.hCam, ueye.IS_GET_COLOR_MODE)
        nRet = ueye.is_AllocImageMem(self.hCam, self.width, self.height, self.nBitsPerPixel, self.pcImageMemory, self.MemID)
        nRet = ueye.is_SetImageMem(self.hCam, self.pcImageMemory, self.MemID)
        nRet = ueye.is_CaptureVideo(self.hCam, ueye.IS_DONT_WAIT)
        nRet = ueye.is_InquireImageMem(self.hCam, self.pcImageMemory, self.MemID, self.width, self.height, self.nBitsPerPixel, self.pitch)
        self.bytes_per_pixel = int(nBitsPerPixel / 8)

    def run(self):
        while True:
            array = ueye.get_data(self.pcImageMemory, self.width, self.height, self.nBitsPerPixel, self.pitch, copy=False)

            frame = np.reshape(array, (self.height.value, self.width.value, self.bytes_per_pixel))
            qImg = QImage(frame, self.width.value, self.height.value, QImage.Format_RGB888)  # Format_Grayscale8
            qImg = qImg.scaled(int(self.width.value / 2), int(self.height.value / 2))
            qpxmp = QPixmap.fromImage(qImg)
            self.send_image.emit(qpxmp)
            if self.stop_thread:
                break
            self.wait(200)

    def stop_thr(self):
        self.stop_thread = True
        Ret = ueye.is_FreeImageMem(self.hCam, self.pcImageMemory, self.MemID)


class Thread_count(QThread):
    """這邊是我使用用來進行is_AddToSequence 加入使用
    這邊我做了一個waitevent 用來等存滿100張圖片到記憶體裡面"""
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
        self.pitch = ueye.INT()

    def run(self):
        width = self.sInfo.nMaxWidth
        height = self.sInfo.nMaxHeight
        # create an event of camera
        ueye.is_EnableEvent(self.hCam, ueye.IS_SET_EVENT_FRAME)
        # ueye.is_FreezeVideo(self.hCam, ueye.IS_DONT_WAIT)
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

                self.array = ueye.get_data(self.m_vpcSeqImgMem[count - 1], self.width, self.height,
                                      24, 12312, copy=False)
                frame = np.reshape(self.array, (self.height.value, self.width.value, 3))
                # ...resize the image by a half
                qImg = QImage(frame, self.width.value, self.height.value, QImage.Format_RGB888) # Format_Grayscale8
                # qImg = qImg.scaled(int(self.width.value / 2), int(self.height.value / 2))
                qpxmp = QPixmap.fromImage(qImg)
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
                # ueye.is_FreezeVideo(self.hCam, ueye.IS_WAIT)
                ueye.is_FreezeVideo(self.hCam, ueye.IS_DONT_WAIT)
                break
            # i = i + 1
        ueye.is_DisableEvent(self.hCam, ueye.IS_SET_EVENT_FRAME)
        if platform.system() == "Windows" and pEvent_FL != None:
            ueye.is_ExitEvent(self.hCam, ueye.IS_SET_EVENT_FRAME)
            # import win32api
            # win32api.CloseHandle(hEvent)
            ctypes.windll.Kernel32.CloseHandle(pEvent_FL)


class ABC:

    def __init__(self, parent=None):
        super(ABC, self).__init__()

        """做相機的設定"""
        ueye.is_SetColorMode(self.hCam, ueye.IS_CM_RGB8_PACKED)
        self.m_viSeqMemID = []
        self.m_vpcSeqImgMem = []

    def _init_ids_camera(self, camera_ids):
        """初始化相機參數"""
        self.hCam = ueye.HIDS(camera_ids)  # 0: first available camera;  1-254: The camera with the specified camera ID
        self.sInfo = ueye.SENSORINFO()
        self.cInfo = ueye.CAMINFO()
        self.pcImageMemory = ueye.c_mem_p()
        self.MemID = ueye.int()
        self.rectAOI = ueye.IS_RECT()
        self.pitch = ueye.INT()
        self.nBitsPerPixel = ueye.INT(24)  # 24: bits per pixel for color mode; take 8 bits per pixel for monochrome
        channels = 3  # 3: channels for color mode(RGB); take 1 channel for monochrome
        self.m_nColorMode = ueye.INT(24)  # Y8/RGB16/RGB24/REG32
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
        self.width = imageSize.s32Width
        self.height = imageSize.s32Height

        # allocate buffer (memory) in a for-loop
        for i in range(0, nmaxbuffer):

            iImageID = ueye.c_int(0)
            pcImhMem = ueye.c_mem_p()
            nRet = ueye.is_AllocImageMem(self.hCam, self.width, self.height, self.nBitsPerPixel, pcImhMem, iImageID)
            if nRet != ueye.IS_SUCCESS:
                break
            nRet = ueye.is_AddToSequence(self.hCam, pcImhMem, iImageID)
            if nRet != ueye.IS_SUCCESS:
                ueye.is_FreeImageMem(self.hCam, pcImhMem, iImageID)
                break
            """做好sequence 後進行thread 拍攝"""
            self.m_viSeqMemID.append(iImageID)
            self.m_vpcSeqImgMem.append(pcImhMem)

        if nRet == ueye.IS_SUCCESS:
            bRet = True
        else:
            bRet = False
        return bRet