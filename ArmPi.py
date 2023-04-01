#!/usr/bin/python3
# coding=utf8
import sys
import os
import cv2
import time
import queue
import Camera
import logging
import threading
import RPCServer
import MjpgServer
import demotest
import numpy as np
import HiwonderSDK.Board as Board
import Functions.Running as Running

logger = logging.getLogger('cv2')
# log输出等级设置
logger.setLevel(logging.CRITICAL)

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

QUEUE_RPC = queue.Queue(10)

def startArmPi():
    global HWEXT, HWSONIC

    RPCServer.QUEUE = QUEUE_RPC
    print('strat Armpi RPC')
    threading.Thread(target=RPCServer.startRPCServer,
                     daemon=True).start()  # rpc服务器
    cam = Camera.Camera()  # 相机读取
    # Running.cam = cam
    while True:
        time.sleep(0.01)
        # 执行需要在本线程中执行的RPC命令

        while True:
            try:
                req, ret = QUEUE_RPC.get(False)
                # print("request=")
                print(req)
                event, params, *_ = ret
                ret[2] = req(params)  # 执行RPC命令
                event.set()
            except:
                break
        #####
        # # 执行功能程序：二维码识别
        try:
            if True:
                if cam.frame is not None:
                    frame = cam.frame.copy()
                    img = demotest.run(frame)
                    # if Running.RunningFunc == 9:
                    #     MjpgServer.img_show = np.vstack((img, frame))
                    # else:                       
                    #     MjpgServer.img_show = img
                else:
                    # MjpgServer.img_show = loading_picture
                    break
            else:
                cam.frame = None
        except KeyboardInterrupt:
            break

if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)
    startArmPi()
