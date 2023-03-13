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
from ids_peak import ids_peak

import platform
import numpy as np
import math
import time
import cv2
import os
import datetime
import ctypes
import struct

# check for install the ids camera sdk
try:
    from pyueye import ueye
except ImportError:
    print("Do not install pyueye")
    print("Please use 'pip install pyueye' to install modules")


class Thread_show_image(QThread):
    send_image = pyqtSignal(QPixmap)

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
        self.flag_save = False

        init_events = ueye.IS_INIT_EVENT()
        init_events.nEvent = ueye.IS_SET_EVENT_FRAME
        init_events.bManualReset = False
        init_events.bInitialState = False
        ueye.is_Event(self.hCam, ueye.IS_EVENT_CMD_INIT, init_events, ueye.sizeof(init_events))

        events = ueye.c_uint(ueye.IS_SET_EVENT_FRAME)
        ueye.is_Event(self.hCam, ueye.IS_EVENT_CMD_ENABLE, events, ueye.sizeof(events))

        self.wait_events = ueye.IS_WAIT_EVENT()
        self.wait_events.nEvent = ueye.IS_SET_EVENT_FRAME
        self.wait_events.nCount = 2
        self.wait_events.nTimeoutMilliseconds = 1000
        self.wait_events.nSignaled = 0
        self.wait_events.nSetCount = 0
        self.bytes_per_pixel = int(nBitsPerPixel / 8)

        nRet = ueye.is_AllocImageMem(self.hCam, width, height, nBitsPerPixel, self.pcImageMemory, self.MemID)

        if nRet != ueye.IS_SUCCESS:
            print("is_AllocImageMem ERROR")
        else:
            # Makes the specified image memory the active memory
            nRet = ueye.is_SetImageMem(self.hCam, self.pcImageMemory, self.MemID)
            if nRet != ueye.IS_SUCCESS:
                print("is_SetImageMem ERROR")
            else:
                # Set the desired color mode
                nRet = ueye.is_SetColorMode(self.hCam, ueye.int(0))

        nRet = ueye.is_CaptureVideo(self.hCam, ueye.IS_DONT_WAIT)
        if nRet != ueye.IS_SUCCESS:
            print("is_CaptureVideo ERROR")

        nRet = ueye.is_InquireImageMem(self.hCam, self.pcImageMemory, self.MemID, width, height, nBitsPerPixel, self.pitch)
        if nRet != ueye.IS_SUCCESS:
            print("is_InquireImageMem ERROR")

    def continue_save_img(self):
        self.tmp = []
        self.flag_save = True
        self.max_img = 5

    def save_img(self):
        for i in range(len(self.tmp)):
            cv2.imwrite(f"{i}.jpg", self.tmp[i])

    def run(self):

        while True:
            ret = ueye.is_Event(self.hCam, ueye.IS_EVENT_CMD_WAIT, self.wait_events, ueye.sizeof(self.wait_events))
            if (ueye.IS_SET_EVENT_FRAME == self.wait_events.nSignaled):
                array = ueye.get_data(self.pcImageMemory, self.width, self.height, self.nBitsPerPixel, self.pitch, copy=False)
                frame = np.reshape(array, (self.height.value, self.width.value, self.bytes_per_pixel))
                # print((self.height.value, self.width.value, self.bytes_per_pixel))
                qImg = QImage(frame, self.width.value, self.height.value, QImage.Format_RGB888)
                qImg = qImg.scaled(int(self.width.value / 2), int(self.height.value / 2))
                qpxmp = QPixmap.fromImage(qImg)
                if self.flag_save:
                    self.tmp.append(frame)
                    if len(self.tmp) > self.max_img:
                        self.flag_save = False
                        self.save_img()
                self.send_image.emit(qpxmp)
                if self.stop_thread:
                    break
                self.wait(200)

    def stop_thr(self):
        self.stop_thread = True
        # Ret = ueye.is_FreeImageMem(self.hCam, self.pcImageMemory, self.MemID)


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
            except ImportError:
                print("error")

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

        self.liveframe = None
        self.bufeersize = None
        # ueye.is_SetColorMode(self.hCam, ueye.IS_CM_RGB8_PACKED)
        self.m_viSeqMemID = []
        self.m_vpcSeqImgMem = []
        # self.signal1.connect(self.update_image)

        # self.image_thread = Thread_show_image(self.hCam, self.width, self.height, self.sInfo, self.nBitsPerPixel)
        # self.image_thread.start()
        # self.image_thread.send_image.connect(self.update_image)

    def _init_ids_camera(self, camera_ids):
        self.hCam = ueye.HIDS(camera_ids)  # 0: first available camera;  1-254: The camera with the specified camera ID
        self.sInfo = ueye.SENSORINFO()
        self.cInfo = ueye.CAMINFO()
        self.pcImageMemory = ueye.c_mem_p()
        self.MemID = ueye.int()
        self.rectAOI = ueye.IS_RECT()
        self.width = ueye.int(4104)   # 相機解析度
        self.height = ueye.int(2174)  # 相機解析度
        self.pitch = ueye.INT()
        self.nBitsPerPixel = ueye.INT(32)  # 24: bits per pixel for color mode; take 8 bits per pixel for monochrome
        channels = 3  # 3: channels for color mode(RGB); take 1 channel for monochrome
        self.m_nColorMode = ueye.INT(0)  # Y8/RGB16/RGB24/REG32
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
        self.image_thread.continue_save_img()
        # self.all_process()

    @pyqtSlot()
    def on_openlive_clicked(self):
        print("open live mode")
        self.image_thread = Thread_show_image(self.hCam, self.width, self.height, self.sInfo, self.nBitsPerPixel)
        self.image_thread.start()
        self.image_thread.send_image.connect(self.update_image)

    @pyqtSlot()
    def on_close_live_clicked(self):
        print("close the live mode")
        self.image_thread.stop_thr()
        self.image_thread.exec_()
        ueye.is_ExitCamera(self.hCam)

    def all_process(self):
        """This function connect all process"""
        pass
        # # Build Camera Sequence
        # self.bufeersize = 100 # self.number_of_shots.value()
        # bRet = self.camSeqBuild()
        #
        # if bRet is True:
        #     print("IS_SUCCESS")
        # else:
        #     self.CamSeqKill()
        #     ueye.is_ExitCamera(self.hCam)
        #     print("Error")
        #
        # # build thread
        # # t = threading.Thread(target=self.job)
        # ueye.is_CaptureVideo(self.hCam, ueye.IS_DONT_WAIT)
        # # ueye.is_CaptureVideo(self.hCam, ueye.IS_DONT_WAIT)
        #
        # self.test_thread = Thread_count(self.hCam, self.width, self.height, self.sInfo, self.m_vpcSeqImgMem,
        #                                 self.bufeersize, self.nBitsPerPixel)
        # self.test_thread.test_signal.connect(self.update_image)
        # self.test_thread.finished.connect(self.savve)
        # self.test_thread.start()

    def savve(self):
        # save image
        folder_name = datetime.datetime.now().strftime("%Y_%m_%d-%H%M")
        # os.mkdir(f"{folder_name}")
        # self.save_img(self.m_vpcSeqImgMem, self.m_viSeqMemID, folder_name)

        self.CamSeqKill()

    def save_img(self, mem, iImageID, folder_name: str):
        print(len(mem))
        plast = ueye.c_mem_p()

        for i in range(0, len(mem)):
            # process the pointer memory translate to image
            # TODO: add process on get data form image

            parameter = ueye.IMAGE_FILE_PARAMS(ppcImageMem=mem[i], pnImageID=iImageID[i])
            parameter.pwchFileName = f"{folder_name}/{i}.jpeg"
            parameter.nQuality = 0
            parameter.nFileType = ueye.IS_IMG_JPG
            ueye.is_ImageFile(self.hCam, ueye.IS_IMAGE_FILE_CMD_SAVE, parameter, ueye.sizeof(parameter))

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

    def closeEvent(self, *args, **kwargs):
        self.image_thread.stop_thr()