import cv2
import glob
import os
import numpy as np
from typing import Tuple
import time


def MatchingMethod(img: np.ndarray, templ: np.ndarray)\
        -> Tuple[float, float]:
    match_method = 3

    methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR',
               'cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']

    result = cv2.matchTemplate(img, templ, method=3)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)

    if match_method == 1 and match_method == 2:
        matchLoc = min_loc
    else:
        matchLoc = max_loc

    return matchLoc
    
def loadimage_process(path: str,
                      files_name: list,
                      merge_padding: int,
                      img_quantity: int,
                      select: int = 0,
                      image_perline: int = 0) \
        -> np.ndarray:
    """
    :param name: is an image path
    :param img_quantity: folder have image quantity
    :return:
    """
    # name = "Lf_2SE_D10_201711221639_"
    # constant variable
    src_img_width = 150
    Tem_img_width = 100
    
    ## 0620 公準 setting
    src_img_width = 500
    Tem_img_width = 450

    MatchingPointBuf = []
    src_img_buffer = []
    Tem_img_buffer = []
    ImgRectRef = 0

    # TODO: change 2054 not an constant
    init_combine_buffer = np.zeros((1024, 1, 3))
    for i in range(1, img_quantity - 1):
        Img_1 = cv2.imread(f"{path}/{files_name[i]}")
        Img_2 = cv2.imread(f"{path}/{files_name[i + 1]}")
        ImgRectRef = int(Img_1.shape[1] / 2 + image_perline)
        src_img = Img_1[0: Img_1.shape[1], int(ImgRectRef): int(ImgRectRef) + src_img_width]
        Tem_img = Img_2[0: Img_2.shape[1], int(ImgRectRef): int(ImgRectRef) + Tem_img_width]
        MatchingPoint = MatchingMethod(src_img, Tem_img)
        src_img_buffer.append(src_img)
        Tem_img_buffer.append(Tem_img)
        MatchingPointBuf.append(MatchingPoint[0])

    # print(MatchingPointBuf)
    # exit()
    # get num number
    # get statistics number
    counts = np.bincount(MatchingPointBuf)
    mode_num = np.argmax(counts)

    if select == 0:
        for i in range(0, img_quantity - 2):
            # use num number to merge
            bottom_right = mode_num + Tem_img_buffer[i].shape[1], 0 + Tem_img_buffer[i].shape[0]
            test = src_img_buffer[i][0:Tem_img_buffer[i].shape[0], merge_padding: merge_padding + mode_num]
            init_combine_buffer = np.hstack((init_combine_buffer, test))
    else:
        for i in range(0, img_quantity - 2):
            bottom_right = MatchingPointBuf[i] + Tem_img_buffer[i].shape[1], 0 + Tem_img_buffer[i].shape[0]
            test = src_img_buffer[i][0:Tem_img_buffer[i].shape[0], merge_padding: merge_padding + MatchingPointBuf[i]]
            init_combine_buffer = np.hstack((init_combine_buffer, test))

    return init_combine_buffer


if __name__ == "__main__":
    # name = glob.glob(f"{folder}/*.bmp")
    start = time.time()
    path = "C:/Users/smpss/kmol/save_img_experiment/2022_06_07_15_46_25"
    files = os.listdir(path)
    files.sort(key=lambda x: os.path.getmtime(f"{path}/{x}"))
    merge_padding = 20
    
    result = loadimage_process(path, files, merge_padding, len(files), select=1, image_perline=0)
    cv2.imwrite("finish_answer.jpg", result)
    end = time.time()

    print(end - start)
    