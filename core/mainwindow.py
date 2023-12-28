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


class CameraThread(QThread):
    update_frame = pyqtSignal(QPixmap)
    update_frame_array = pyqtSignal(np.ndarray)

    def __init__(self, camera_id=0):
        super(CameraThread, self).__init__()
        self.hCam = ueye.HIDS(camera_id)
        self.running = False

    def run(self):
        self.init_camera()
        self.running = True

        while self.running:
            # frame = self.capture_frame()
            ret = ueye.is_Event(self.hCam, ueye.IS_EVENT_CMD_WAIT, self.wait_events, ueye.sizeof(self.wait_events))
            if (ueye.IS_SET_EVENT_FRAME == self.wait_events.nSignaled) or frame is not None:
                # ...extract the data of our image memory
                array = ueye.get_data(self.pcImageMemory, self.width, self.height, self.nBitsPerPixel, self.pitch, copy=False)
                # bytes_per_pixel = int(nBitsPerPixel / 8)
                # ...reshape it in an numpy array...
                frame = np.reshape(array, (self.height.value, self.width.value, self.bytes_per_pixel))
                qImg = self.convert_to_qimage(frame)
                qpxmp = QPixmap.fromImage(qImg)

                self.update_frame.emit(qpxmp)
                self.update_frame_array.emit(frame)

        self.stop_camera()

    def convert_to_qimage(self, frame):
        height, width, channels = frame.shape
        bytes_per_line = channels * width
        q_image = QImage(frame.data, width, height, bytes_per_line, QImage.Format_RGB32)
        # q_image.save("test.jpg")
        return q_image

    def init_camera(self):

        hCam = self.hCam  # 0: first available camera;  1-254: The camera with the specified camera ID
        sInfo = ueye.SENSORINFO()
        cInfo = ueye.CAMINFO()
        pcImageMemory = ueye.c_mem_p()
        MemID = ueye.int()
        rectAOI = ueye.IS_RECT()
        pitch = ueye.INT()
        nBitsPerPixel = ueye.INT(24)  # 24: bits per pixel for color mode; take 8 bits per pixel for monochrome
        channels = 3  # 3: channels for color mode(RGB); take 1 channel for monochrome
        m_nColorMode = ueye.INT()  # Y8/RGB16/RGB24/REG32
        bytes_per_pixel = int(nBitsPerPixel / 8)
        now = ctypes.c_uint()
        # =ueye.UEYE_AUTO_INFO()

        # ---------------------------------------------------------------------------------------------------------------------------------------
        print("START")
        print()

        # Starts the driver and establishes the connection to the camera
        nRet = ueye.is_InitCamera(hCam, None)
        if nRet != ueye.IS_SUCCESS:
            print("is_InitCamera ERROR")

        # Reads out the data hard-coded in the non-volatile camera memory and writes it to the data structure that cInfo points to
        nRet = ueye.is_GetCameraInfo(hCam, cInfo)
        if nRet != ueye.IS_SUCCESS:
            print("is_GetCameraInfo ERROR")

        # You can query additional information about the sensor type used in the camera
        nRet = ueye.is_GetSensorInfo(hCam, sInfo)
        if nRet != ueye.IS_SUCCESS:
            print("is_GetSensorInfo ERROR")

        nRet = ueye.is_ResetToDefault(hCam)
        if nRet != ueye.IS_SUCCESS:
            print("is_ResetToDefault ERROR")

        # Set display mode to DIB
        nRet = ueye.is_SetDisplayMode(hCam, ueye.IS_SET_DM_DIB)

        # Set the right color mode
        if int.from_bytes(sInfo.nColorMode.value, byteorder='big') == ueye.IS_COLORMODE_BAYER:
            # setup the color depth to the current windows setting
            ueye.is_GetColorDepth(hCam, nBitsPerPixel, m_nColorMode)
            bytes_per_pixel = int(nBitsPerPixel / 8)
            print("IS_COLORMODE_BAYER: ", )
            print("\tm_nColorMode: \t\t", m_nColorMode)
            print("\tnBitsPerPixel: \t\t", nBitsPerPixel)
            print("\tbytes_per_pixel: \t\t", bytes_per_pixel)
            print()

        elif int.from_bytes(sInfo.nColorMode.value, byteorder='big') == ueye.IS_COLORMODE_CBYCRY:
            # for color camera models use RGB32 mode

            nBitsPerPixel = ueye.INT(32)
            bytes_per_pixel = int(nBitsPerPixel / 8)
            print("IS_COLORMODE_CBYCRY: ", )
            print("\tm_nColorMode: \t\t", m_nColorMode)
            print("\tnBitsPerPixel: \t\t", nBitsPerPixel)
            print("\tbytes_per_pixel: \t\t", bytes_per_pixel)
            print()

        elif int.from_bytes(sInfo.nColorMode.value, byteorder='big') == ueye.IS_COLORMODE_MONOCHROME:
            # for color camera models use RGB32 mode
            m_nColorMode = ueye.IS_CM_MONO8
            nBitsPerPixel = ueye.INT(8)
            bytes_per_pixel = int(nBitsPerPixel / 8)
            print("IS_COLORMODE_MONOCHROME: ", )
            print("\tm_nColorMode: \t\t", m_nColorMode)
            print("\tnBitsPerPixel: \t\t", nBitsPerPixel)
            print("\tbytes_per_pixel: \t\t", bytes_per_pixel)
            print()

        else:
            # for monochrome camera models use Y8 mode
            m_nColorMode = ueye.IS_CM_MONO8
            nBitsPerPixel = ueye.INT(8)
            bytes_per_pixel = int(nBitsPerPixel / 8)
            print("else")

        # Can be used to set the size and position of an "area of interest"(AOI) within an image
        nRet = ueye.is_AOI(hCam, ueye.IS_AOI_IMAGE_GET_AOI, rectAOI, ueye.sizeof(rectAOI))
        if nRet != ueye.IS_SUCCESS:
            print("is_AOI ERROR")

        width = rectAOI.s32Width
        height = rectAOI.s32Height

        # Prints out some information about the camera and the sensor
        print("Camera model:\t\t", sInfo.strSensorName.decode('utf-8'))
        print("Camera serial no.:\t", cInfo.SerNo.decode('utf-8'))
        print("Maximum image width:\t", width)
        print("Maximum image height:\t", height)
        print()

        # ---------------------------------------------------------------------------------------------------------------------------------------

        # Allocates an image memory for an image having its dimensions defined by width and height and its color depth defined by nBitsPerPixel
        nRet = ueye.is_AllocImageMem(hCam, width, height, nBitsPerPixel, pcImageMemory, MemID)
        print(hCam, width, height, nBitsPerPixel, pcImageMemory, MemID)
        if nRet != ueye.IS_SUCCESS:
            print("is_AllocImageMem ERROR")
        else:
            # Makes the specified image memory the active memory
            nRet = ueye.is_SetImageMem(hCam, pcImageMemory, MemID)
            if nRet != ueye.IS_SUCCESS:
                print("is_SetImageMem ERROR")
            else:
                # Set the desired color mode
                nRet = ueye.is_SetColorMode(hCam, m_nColorMode)
                # nRet = ueye.is_SetColorMode(self.hCam, ueye.IS_CM_BGRA8_PACKED)

        # Activates the camera's live video mode (free run mode)
        nRet = ueye.is_CaptureVideo(hCam, ueye.IS_DONT_WAIT)
        if nRet != ueye.IS_SUCCESS:
            print("is_CaptureVideo ERROR")

        nRet = ueye.is_InquireImageMem(hCam, pcImageMemory, MemID, width, height, nBitsPerPixel, pitch)

        if nRet != ueye.IS_SUCCESS:
            print("is_InquireImageMem ERROR")
        else:
            print("Press q to leave the programm")

        init_events = ueye.IS_INIT_EVENT()
        init_events.nEvent = ueye.IS_SET_EVENT_FRAME
        init_events.bManualReset = False
        init_events.bInitialState = False
        ueye.is_Event(hCam, ueye.IS_EVENT_CMD_INIT, init_events, ueye.sizeof(init_events))

        events = ueye.c_uint(ueye.IS_SET_EVENT_FRAME)
        ueye.is_Event(hCam, ueye.IS_EVENT_CMD_ENABLE, events, ueye.sizeof(events))

        self.wait_events = ueye.IS_WAIT_EVENT()
        self.wait_events.nEvent = ueye.IS_SET_EVENT_FRAME
        self.wait_events.nCount = 2
        self.wait_events.nTimeoutMilliseconds = 1000
        self.wait_events.nSignaled = 0
        self.wait_events.nSetCount = 0

        # pcImageMemory, width, height, nBitsPerPixel, pitch
        self.width = width
        self.height = height
        self.pcImageMemory = pcImageMemory
        self.nBitsPerPixel = nBitsPerPixel
        self.pitch = pitch
        self.bytes_per_pixel = bytes_per_pixel
        self.MemID = MemID

    def stop_camera(self):
        # Releases an image memory that was allocated using is_AllocImageMem() and removes it from the driver management
        net = ueye.is_FreeImageMem(self.hCam, self.pcImageMemory, self.MemID)
        print(net)

        # Disables the hCam camera handle and releases the data structures and memory areas taken up by the uEye camera
        net = ueye.is_ExitCamera(self.hCam)
        print(net)

    def stop(self):
        self.running = False


class MainWindow(QMainWindow):

    signal1 = pyqtSignal()

    def __init__(self, parent = None):
        super(MainWindow, self).__init__()
        loadUi("core/mainwindow.ui", self)
        # self.__init_setting()
        # self._add_action()
        self.show()
        self.current_image = None


    @pyqtSlot()
    def on_testbtn_clicked(self):
        print("current data")
        print(self.current_image.shape)
        print(self.current_image)
        cv2.imwrite("test.jpg", self.current_image)

    @pyqtSlot()
    def on_openlive_clicked(self):
        print("open live mode")
        self.camera_thread = CameraThread(0)
        self.camera_thread.update_frame.connect(self.update_image)
        self.camera_thread.update_frame_array.connect(self.update_img_data)
        self.camera_thread.start()

        print("Process")

    @pyqtSlot()
    def on_close_live_clicked(self):
        print("close the live mode")
        self.camera_thread.stop()
        print("Close event")


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

    def update_image(self, data):
        self.image1.setPixmap(data)
        self.image1.setScaledContents(True)

    def update_img_data(self, img_data):
        self.current_image = img_data

    def closeEvent(self, *args, **kwargs):
       if self.camera_thread.isRunning():
           self.camera_thread.stop()
       else:
           pass

