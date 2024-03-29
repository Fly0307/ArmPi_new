#!/usr/bin/python3
# coding=utf8
import sys
import time
import threading
import QRcodeIdentify
import demotest
import Functions.RemoteControl as RemoteControl


RunningFunc = 0
LastHeartbeat = 0
cam = None

FUNCTIONS = {
    1: RemoteControl,
    2: QRcodeIdentify,    # 启动二维码识别货物
    3: demotest,    # 
    4: None,
}

def doHeartbeat(tmp=()):
    global LastHeartbeat
    LastHeartbeat = time.time() + 7
    return (True, ())

def CurrentEXE():
    global RunningFunc
    return FUNCTIONS[RunningFunc]

def loadFunc(newf):
    global RunningFunc
    new_func = newf[0]
    doHeartbeat()
    if new_func < 1 or new_func > 9:
        return (False, sys._getframe().f_code.co_name + ": Invalid argument")
    else:
        print(f'RuningFunc={RunningFunc}')
        try:
            if RunningFunc > 1:
                FUNCTIONS[RunningFunc].exit()
            RunningFunc = newf[0]
            # cam.camera_close()
            # cam.camera_open()
            FUNCTIONS[RunningFunc].init()
        except Exception as e:
            print(e)
    return (True, (RunningFunc,))

def unloadFunc(tmp = ()):
    global RunningFunc
    if RunningFunc != 0:
        FUNCTIONS[RunningFunc].exit()
        RunningFunc = 0
    cam.camera_close()
    return (True, (0,))

def getLoadedFunc(newf):
    global RunningFunc
    return (True, (RunningFunc,))

def startFunc(tmp):
    global RunningFunc
    FUNCTIONS[RunningFunc].start()
    return (True, (RunningFunc,))

def stopFunc(tmp):
    global RunningFunc
    FUNCTIONS[RunningFunc].stop()
    return (True, (RunningFunc,))

def heartbeatTask():
    return
    global LastHeartbeat
    global RunningFunc
    while True:
        try:
            if LastHeartbeat < time.time():
                if RunningFunc != 0:
                    unloadFunc()
            time.sleep(0.1)
        except KeyboardInterrupt:
            break

threading.Thread(target=heartbeatTask, daemon=True).start()