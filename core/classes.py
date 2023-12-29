# -*- coding: utf-8 -*-

__author__ = "Yu-Sheng Lin"
__copyright__ = "Copyright (C) 2016-2022"
__license__ = "AGPL"
__email__ = "pyquino@gmail.com"

import numpy as np
import cv2
import time
import os
import ctypes as C

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from core.serial_core.serialportcontext import SerialPortContext
from core.template_matching import loadimage_process
from core.ai_detected.run_model import LoadAIModel


def converte_pixmap2array(dispBuffer):

    channels_count = 4
    # dispBuffer.pixmap.save("test.jpg")
    image = dispBuffer.pixmap.toImage()
    image = image.convertToFormat(QImage.Format.Format_RGBA8888)
    size = image.size()
    s = image.bits().asstring(size.width() * size.height() * image.depth() // 8)  # format 0xffRRGGBB
    arr = np.fromstring(s, dtype=np.uint8).reshape((size.height(), size.width(), image.depth() // 8))
    R, G, B, D = cv2.split(arr)
    BGR_image = cv2.merge([B, G, R])
    return BGR_image


def conver_qimage2array(img: QImage):

    image = img.convertToFormat(QImage.Format.Format_RGBA8888)
    size = image.size()
    s = image.bits().asstring(size.width() * size.height() * image.depth() // 8)  # format 0xffRRGGBB
    arr = np.fromstring(s, dtype=np.uint8).reshape((size.height(), size.width(), image.depth() // 8))
    R, G, B, D = cv2.split(arr)
    BGR_image = cv2.merge([B, G, R])
    return BGR_image


class Thread_slect_focus(QThread):
    classifier_img = pyqtSignal(object)

    def __init__(self, send_thread_handle, use_ai=False, parent=None):
        QThread.__init__(self, parent)
        self.send_thread = send_thread_handle
        self.image_arr = None
        self.laplacian = None
        self.tet = None
        # self.commads = ["G28 X0\n", "G28 Y0\n", "G28 Z0\n",
        #                 "G1X9.0Y95.0Z26.0F300\nM114\n"]
        # self.loadai = LoadAIModel("31_tool_knife.pth")
        self.use_ai = use_ai

    def run(self):
        # Notice command need [command] list type
        # X:9.0 Y95.0 Z5.0   Z->26 -> 5
        # G28 X0
        # G28 Y0
        # G28 Z0
        # G1X9.0Y95.0Z26.0F300
        tmp = []

        # test for continue get image from array
        data = f"G91\nG1E1300F{1000}\nG90\nM114\n"
        # 透過 send thread motion_step 進行指令傳送
        # self.send_thread.motion_step = [data]
        # self.send_thread.start()
        print("傳送指令開始")
        print(data)
        self.msleep(200)
        t = time.localtime()
        current_time = time.strftime("%Y_%m_%d_%H_%M_%S", t)
        folder_path = f"tmp/{current_time}"
        os.mkdir(folder_path)
        for i in range(30):
            # print(self.send_thread.image_arr)
            print(f"儲存照片 {i} 張")
            img = self.send_thread.image_arr
            cv2.imwrite(f"{folder_path}/{i:03d}.bmp", img)
            # self.tet.pixmap.save(f"{folder_path}/{i:03d}.bmp")
            # cv2.imwrite(f"{folder_path}/{i:03d}.bmp", self.image_arr)
            self.msleep(200)

        # files = os.listdir(folder_path)
        # files.sort(key=lambda x: os.path.getmtime(f"{folder_path}/{x}"))
        merge_padding = 10

        # result = loadimage_process(folder_path, files, merge_padding, len(files), select=0, image_perline=0)
        # filp_image = cv2.flip(result, 0)
        # filp_image = cv2.flip(filp_image, 1)
        # filp_image = cv2.cvtColor(filp_image, cv2.COLOR_BGR2GRAY)
        # cv2.imwrite(f"C:/Users/smpss/kmol/save_img_experiment/template_matching_merge/{current_time}.jpg", filp_image)
        # self.send_thread.motion_step = self.commads
        # self.send_thread.start()
        # self.send_thread.wait()
        #
        # for i in range(300):
        #     data = f"G91\nG1E5F{1000}\nG90\nM114\n"
        #     # print(data)
        #     # self.send_thread.motion_step = [data]
        #     # self.send_thread.start()
        #     # self.send_thread.wait()
        #     cv2.imwrite(f"C:/Users/smpss/kmol/save_img_experiment/new_tool/{i}.bmp", self.image_arr)
        #     self.msleep(200)

        # for i in range(21):
        #     cmd = [f"G1X9.0Y95.0Z{26 - i}F300\nM114\n"]
        #     self.send_thread.motion_step = cmd
        #     self.send_thread.start()
        #     self.send_thread.wait()
        #     tmp.append(self.laplacian)
        #     if self.use_ai and self.laplacian > 10:
        #         detected_img = self.loadai.run_model_method(self.image_arr)
        #         self.classifier_img.emit(detected_img)

        # best_locate in detected knife
        # cmd = [f"G1X9.0Y95.0Z{26 - tmp.index(max(tmp))}F300\nM114\n"]
        # self.send_thread.motion_step = cmd
        # self.send_thread.start()
        # self.send_thread.wait()
        # while cnt < 15:
        #
        #     self.send_thread.motion_step = [10]
        #     self.send_thread.start()
        #     self.send_thread.wait()
        #     tmp.append(self.laplacian)
        #     cnt = cnt + 1
        #
        #     self.msleep(100)
        # self.send_thread.motion_step = [tmp.index(max(tmp)) - 15]
        # self.send_thread.start()
        # self.send_thread.wait()

    def get_laplacin_value(self, image, laplacian):
        self.laplacian = laplacian
        self.image_arr = image
        # print(laplacian)


class Thread_scale_image(QThread):
    """這邊會透過設計AI用來進行自動辨識"""
    classifier_img = pyqtSignal(object)

    def __init__(self, send_thread_handle, parent=None):
        QThread.__init__(self, parent)
        self.send_thread = send_thread_handle
        self.image_arr = None
        self.laplacian = None
        self.waitSignal = False
        self.use_ai = False
        # self.loadai = LoadAIModel("31_tool_knife.pth")

    def use_ai_detected(self):
        self.use_ai = True

    def run(self):
        for i in range(0, 181, 30):
            self.send_thread.scale_command(i)
            while True:
                if self.waitSignal:
                    if self.use_ai:
                        detected_img = self.loadai.run_model_method(self.image_arr)
                        self.classifier_img.emit(detected_img)
                    # cv2.imwrite(f"tmp_img/autozoom/{i:02d}.jpg", self.image_arr)
                    break
                self.msleep(300)
            self.waitSignal = False
        for i in range(180, -1, -30):
            self.send_thread.scale_command(i)

    def get_image(self, image):
        self.image_arr = image
        self.waitSignal = True


class Thread_wait_forController(QThread):
    lnc_signal = pyqtSignal(int)
    laplacian_signal = pyqtSignal(object, object)
    zoomcommand_signal = pyqtSignal(object)

    def __init__(self, parent=None):
        QThread.__init__(self, parent)
        self.flag = True
        self.t0 = 0
        self.image_arr = None
        self.motion_step = None
        self.serial_hadle = None
        self.status_flag = False
        self.staMode = False

    def run(self):
        i = 0
        if self.serial_hadle.isRunning():
            while i < len(self.motion_step):
                self.ramps_command(self.motion_step[i])
                # text = f"$J=G21G91Y{self.motion_step[i]}F250"                
                # self.__test__send(self.serial_hadle, f"{text}")  # this line is command
                # self.__test__send(self.serial_hadle, "M114")
                while True:
                    if self.status_flag:
                        self.laplacian_signal.emit(self.image_arr, self.Laplacina(self.image_arr))
                        break
                    else:
                        self.msleep(500)
                self.msleep(200)
                self.status_flag = False
                self.staMode = False
                i += 1

    def scale_command(self, scale):
        self.__test__send(self.serial_hadle, f"M998 P0 S{scale}")
        self.msleep(500)
        self.zoomcommand_signal.emit(self.image_arr)
    
    def ramps_command(self, command):

        # self.__test__send(self.serial_hadle, "G91")
        # self.__test__send(self.serial_hadle, f"G1 Y{move_distance} F500")
        # self.__test__send(self.serial_hadle, "G90")
        # self.__test__send(self.serial_hadle, "M114")
        self.__test__send(self.serial_hadle, command)
        self.staMode = True

    def __test__send(self, contex, data1):
        data = str(data1 + '\n')
        if contex.isRunning():
            if len(data) > 0:
                contex.send(data, 0)

    def test_received(self, data):
        if data.startswith("echo:busy") or self.staMode:
            if data.startswith("X:"):
                # print(data)
                # x, y, z = data.split("|")[1].split(":")[1].split(",")
                # print(f"X:{x} Y:{y} Z:{z}")
                self.status_flag = True
            # self.sleep(5000)
            # self.status_flag = True

        elif data.startswith("<Run|"):
            pass

    # def aaa(self):
    #     self.__test__send(self.serial_hadle, "?")

    def motion(self, motion1: list):
        self.motion_step = motion1

    def get_serial_handle(self, serial_handle):
        self.serial_hadle = serial_handle

    def inputimage(self, pixmap):
        self.image_arr = pixmap
        # print(self.image_arr.shape)

    def Laplacina(self, img):
        return cv2.Laplacian(img, cv2.CV_64F).var()


class SinkData:
    brightnes = 0
    FrameBuffer = None


class DisplayBuffer:
    '''
    This class is needed to copy the image into a pixmap for
    displaying in the video window.
    '''
    locked = False
    pixmap = None
    img = None

    def Copy(self, FrameBuffer):
        if (int(FrameBuffer.FrameType.BitsPerPixel / 8) == 4):
            imgcontent = C.cast(FrameBuffer.GetIntPtr().ToInt64(),
                                C.POINTER(C.c_ubyte * FrameBuffer.FrameType.BufferSize))
            qimage = QImage(imgcontent.contents, FrameBuffer.FrameType.Width, FrameBuffer.FrameType.Height,
                            QImage.Format_RGB32).mirrored()
            self.img = qimage
            self.pixmap = QPixmap(qimage)


class WorkerSignals(QObject):
    display = pyqtSignal(object)

#
# class DisplayFilter(TIS.Imaging.FrameFilterImpl):
#     '''
#     This frame filter copies an incoming frame into our
#     DisplayBuffer object and signals the QApplication
#     with the new buffer.
#     '''
#     __namespace__ = "DisplayFilterClass"
#     signals = WorkerSignals()
#     dispBuffer = DisplayBuffer()
#
#     def GetSupportedInputTypes(self, frameTypes):
#         frameTypes.Add(TIS.Imaging.FrameType(TIS.Imaging.MediaSubtypes.RGB32))
#
#     def GetTransformOutputTypes(self, inType, outTypes):
#         outTypes.Add(inType)
#         return True
#
#     def Transform(self, src, dest):
#         dest.CopyFrom(src)
#         if self.dispBuffer.locked is False:
#             self.dispBuffer.locked = True
#             self.dispBuffer.Copy(dest)
#             self.signals.display.emit(self.dispBuffer)
#
#         return False