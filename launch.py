# -*- coding: utf-8 -*-

import sys
import os
from PyQt5.QtWidgets import *

if __name__ == "__main__":
    from core.mainwindow import MainWindow

    app = QApplication(sys.argv)
    window = MainWindow()
    app.exec_()