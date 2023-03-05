#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import Functions.Running as Running
import HiwonderSDK.Board as Board
from ArmIK.ArmMoveIK import *
from jsonrpc import JSONRPCResponseManager, dispatcher
from werkzeug.serving import run_simple
from werkzeug.wrappers import Request, Response
# import QRcodeIdentify
import threading
import logging
import time
import os


if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

__RPC_E01 = "E01 - Invalid number of parameter!"
__RPC_E02 = "E02 - Invalid parameter!"
__RPC_E03 = "E03 - Operation failed!"
__RPC_E04 = "E04 - Operation timeout!"
__RPC_E05 = "E05 - Not callable"

QUEUE = None

# 夹持器夹取时闭合的角度
servo1 = 500
AK=ArmIK()
# 初始位置
def initMove():
    print('init move')
    Board.setBusServoPulse(1, servo1 - 50, 300)
    Board.setBusServoPulse(2, 500, 500)
    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
    print("Init finished")
    
initMove()

#设置PWM伺服脉冲
@dispatcher.add_method
def SetPWMServo(*args, **kwargs):
    ret = (True, (), 'SetPWMServo')
    arglen = len(args)
    if 0 != (arglen % 3):
        return (False, __RPC_E01, 'SetPWMServo')
    try:
        servos = args[0:arglen:3]
        pulses = args[1:arglen:3]
        use_times = args[2:arglen:3]
        for s in servos:
            if s < 1 or s > 6:
                return (False, __RPC_E02)
        dat = zip(servos, pulses, use_times)
        for (s, p, t) in dat:
            Board.setPWMServoPulse(s, p, t)
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'SetPWMServo')
    return ret

#设置总线伺服脉冲
@dispatcher.add_method
def SetBusServoPulse(*args, **kwargs):
    ret = (True, (), 'SetBusServoPulse')
    arglen = len(args)
    if (args[1] * 2 + 2) != arglen or arglen < 4:
        return (False, __RPC_E01, 'SetBusServoPulse')
    try:
        servos = args[2:arglen:2]
        pulses = args[3:arglen:2]
        use_times = args[0]
        for s in servos:
            if s < 1 or s > 6:
                return (False, __RPC_E02)
        dat = zip(servos, pulses)
        for (s, p) in dat:
            Board.setBusServoPulse(s, p, use_times)
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'SetBusServoPulse')
    return ret


#掉电
@dispatcher.add_method
def UnloadBusServo(args):
    ret = (True, (), 'UnloadBusServo')
    if args != 'servoPowerDown':
        return (False, __RPC_E01, 'UnloadBusServo')
    try:
        for i in range(1, 7):
            Board.unloadBusServo(i)
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'UnloadBusServo')

#读取舵机当前位置
@dispatcher.add_method
def GetBusServosPulse(args):
    print(f'GetBusServosPulse({args})')
    ret = (True, (), 'GetBusServosPulse')
    data = []
    if args != 'angularReadback':
        return (False, __RPC_E01, 'GetBusServosPulse')
    try:
        for i in range(1, 7):
            pulse = Board.getBusServoPulse(i)
            if pulse is None:
                ret = (False, __RPC_E04, 'GetBusServosPulse')
                return ret
            else:
                data.append(pulse)
        ret = (True, data, 'GetBusServosPulse')
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'GetBusServosPulse')
    return ret


@dispatcher.add_method
def ArmMoveIk(*args):
    """ 
    控制机械臂运行，args记录
    坐标coordinate_data和俯仰角alpha,
    以及俯仰角范围的范围alpha1, alpha2，自动寻找最接近给定俯仰角的解
    """
    ret = (True, (), 'ArmMoveIk')
    if len(args) != 7:
        return (False, __RPC_E01, 'ArmMoveIk')
    try:
        result = setPitchRangeMoving(
            (args[0], args[1], args[2]), args[3], args[4], args[5], args[6])
        ret = (True, result)
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'ArmMoveIk')
    return ret

#设置电刷电机
@dispatcher.add_method
def SetBrushMotor(*args, **kwargs):
    ret = (True, (), 'SetBrushMotor')
    arglen = len(args)
    if 0 != (arglen % 2):
        return (False, __RPC_E01, 'SetBrushMotor')
    try:
        motors = args[0:arglen:2]
        speeds = args[1:arglen:2]
        for m in motors:
            if m < 1 or m > 4:
                return (False, __RPC_E02)
        dat = zip(motors, speeds)

        for m, s in dat:
            Board.setMotor(m, s)
    except:
        ret = (False, __RPC_E03, 'SetBrushMotor')
    return ret



#获取电池电压
@dispatcher.add_method
def GetBatteryVoltage():
    ret = (True, 0, 'GetBatteryVoltage')
    try:
        ret = (True, Board.getBattery(), 'GetBatteryVoltage')
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'GetBatteryVoltage')
    return ret


def runbymainth(req, pas):
    """ 调用 """
    if callable(req):
        event = threading.Event()
        ret = [event, pas, None]
        QUEUE.put((req, ret))
        count = 0
        # ret[2] =  req(pas)
        # print('ret', ret)
        #2s以上未返回则超时
        while ret[2] is None:
            time.sleep(0.01)
            count += 1
            if count > 200:
                break
        if ret[2] is not None:
            if ret[2][0]:
                return ret[2]
            else:
                return (False, __RPC_E03 + " " + ret[2][1])
        else:
            return (False, __RPC_E04)
    else:
        return (False, __RPC_E05)


@dispatcher.add_method
def LoadFunc(new_func=0):
    """ 传入函数号，启动 """
    return runbymainth(Running.loadFunc, (new_func, ))


@dispatcher.add_method
def UnloadFunc():
    return runbymainth(Running.unloadFunc, ())


@dispatcher.add_method
def StartFunc():
    return runbymainth(Running.startFunc, ())


@dispatcher.add_method
def StopFunc():
    return runbymainth(Running.stopFunc, ())


@dispatcher.add_method
def Heartbeat():
    return runbymainth(Running.doHeartbeat, ())

#心跳检测 识别抓取
@dispatcher.add_method
def ArmHeartbeat(alive):
    print('ArmHeartbeat(alive)')
    return runbymainth(QRcodeIdentify.Heartbeat, alive)

@dispatcher.add_method
def GetOrderId():
    print('GetOrderId()')
    return runbymainth(QRcodeIdentify.setTarget, ('None'))

# 放置
@dispatcher.add_method
def CargoPlacement(*target_pos):
    print('CargoPlacement(*target_pos)')
    return runbymainth(QRcodeIdentify.setTarget, target_pos)


@Request.application
def application(request):
    '''
    服务的主方法，handle里面的dispatcher就是代理的rpc方法，可以写多个dispatcher
    :param request: 
    :return: 
    '''
    dispatcher["echo"] = lambda s: s
    dispatcher["add"] = lambda a, b: a + b
    # print(request.data)
    response = JSONRPCResponseManager.handle(request.data, dispatcher)
    return Response(response.json, mimetype='application/json')

# 在本地主机的 9030 端口上启动服务器，并等待来自客户端的请求


def startRPCServer():
    #    log = logging.getLogger('werkzeug')
    #    log.setLevel(logging.ERROR)
    # run_simple('',8090,application)
    run_simple('', 9030, application)


if __name__ == '__main__':
    startRPCServer()