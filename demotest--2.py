import cv2
import time
import threading
import sys
from RPCClient import *
from Camera import *

sys.path.append("/home/pi/ArmPi/")
# sys.path.append('/home/pi/.local/lib/python3.7/site-packages')
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from math import *
import pyzbar.pyzbar as pyzbar

if sys.version_info.major == 2:
    print("Please run this program with python3!")
    sys.exit(0)

# 调用机械臂控制函数
AK = ArmIK()
rpcclient = RPCClient("http://192.168.0.103:9030")
# 放置位置,代表地址
# 坐标以机械臂底座舵机为中心
coordinate = {
    "成都": (-15 + 0.5, 6 - 0.5, 2),
    "天津": (-15 + 0.5, -0 - 0.5, 2),
    "上海": (-15 + 0.5, -6 - 0.5, 2),
}

size = (640, 480)  # 设置采集图片(流)大小

# 夹持器夹取时闭合的角度
servo1 = 500

ArmPi_id = 2
destination = None
cur_orderid = None
# 放置高度计数
count = {
    "成都": 0,
    "天津": 0,
    "上海": 0,
}

orderblocks = {}
orderIDs = set()


def query_order(orderid):
    global orderblocks
    if orderid in orderblocks:
        return orderblocks[orderid]
    else:
        return None

def insert_order(orderid, position, angle):
    global orderblocks
    orderblocks[orderid] = (position, angle)

def update_order(orderid, position, angle):
    global orderblocks
    if orderid in orderblocks:
        orderblocks[orderid] = (position, angle)
    else:
        print("Order not found.")

def delete_order(orderid):
    global orderblocks
    if orderid in orderblocks:
        del orderblocks[orderid]
    else:
        print("Order not found.")



def initMove():
    """初始化位置"""
    print("QR initmove()")
    Board.setBusServoPulse(1, servo1 - 50, 300)
    Board.setBusServoPulse(2, 500, 500)
    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1000)


def setBuzzer(timer):
    """
    蜂鸣器提示
    """
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)


def move():
    global _stop
    global get_roi
    global move_square
    global unreachable
    global destination
    global rpcclient
    global count
    global cur_orderid
    # global start_pick_up
    # global start_pick_down
    global pick_up
    global rotation_angle
    global world_X, world_Y
    global ArmPi_id
    global z_r, z_g, z_b, z

    while True:
        startTime = 0
        if cur_orderid == None:
            time.sleep(0.01)
            continue

        print(f"detect_block={destination}")
        get_it = False
        count_num = 0
        while not get_it:
            startTime = time.perf_counter()
            if destination is not None:
                # 如果抓取2次后还未抓取成功，则清空识别记录重新识别
                if count_num > 1:
                    # start_pick_up = False
                    # start_pick_down = False
                    if(cur_orderid in orderIDs):
                        orderIDs.remove(cur_orderid)
                    delete_order(cur_orderid)
                    cur_orderid=None
                    destination=None
                    initMove()
                    break
                # setBuzzer(0.1)
                print("move to world_X=%d" % (world_X) + "and world_Y=%d" % (world_Y))
                result = AK.setPitchRangeMoving(
                    (world_X, world_Y, 7), -90, -90, 0, 1500
                )  # 移到目标位置上方，高度5cm

                time.sleep(result[2] / 1000)
                if result == False:
                    unreachable = True
                else:
                    unreachable = False
                    
                    # 计算夹持器需要旋转的角度
                    servo2_angle = getAngle(world_X, world_Y, rotation_angle)
                    Board.setBusServoPulse(2, servo2_angle, 300)  # 旋转爪子
                    time.sleep(0.3)

                    
                    result = AK.setPitchRangeMoving(
                        (world_X, world_Y, 5), -90, -90, 0, 1000
                    )  # 降低高度到4cm
                    time.sleep(result[2] / 1000)
                    if result == False:
                        print("can't reach")
                    Board.setBusServoPulse(1, servo1 - 280, 300)  # 爪子张开
                    time.sleep(0.3)
                    
                    result = AK.setPitchRangeMoving(
                        (world_X, world_Y, 1.5), -90, -90, 0, 500
                    )  # 降低高度到2cm
                    time.sleep(result[2] / 1000)

                    
                    Board.setBusServoPulse(1, servo1, 300)  # 夹持器闭合
                    time.sleep(0.3)

                    
                    Board.setBusServoPulse(2, 500, 300)
                    result = AK.setPitchRangeMoving(
                        (world_X, world_Y, 9), -90, -90, 0, 1000
                    )  # 机械臂抬起
                    time.sleep(result[2] / 1000)
                    servo1_now = Board.getBusServoPulse(1)
                    # print("servo1_now=%d"%servo1_now)
                    # 未夹取成功
                    if servo1_now >= 490:
                        print("don't get it")
                        count_num += 1
                        if cur_orderid == None:
                            # start_pick_up = False
                            # start_pick_down = False
                            destination = None
                            initMove()
                        continue
                    else:
                        get_it = True
                    result = AK.setPitchRangeMoving(
                        (0, 10, 12), -90, -90, 0, 1000
                    )  # 机械臂抬起
                    time.sleep(result[2] / 1000)
                    pick_up = True
                    # start_pick_up = False
                    print("机械臂抬起")
            else:
                get_it = False
                
        if destination == None:
            time.sleep(0.01)
            continue
        res = rpcclient.call("Update_state", ArmPi_id)
        while get_it and cur_orderid is not None:
            # 删除订单信息
            orderIDs.remove(cur_orderid)
            delete_order(cur_orderid)
            if destination is not None:
                print("机械臂开始放下 detect_block=%s" % destination)
                pick_up = False
                # start_pick_up = False
                # if not __isRunning:
                #     continue
                print(f"destination={destination}")
                result = AK.setPitchRangeMoving(
                    (coordinate[destination][0], coordinate[destination][1], 12),
                    -90,
                    -90,
                    0,
                    1000,
                )
                time.sleep(result[2] / 1000)
                print(result)

                # if not __isRunning:
                #     continue
                result = AK.setPitchRangeMoving(
                    (
                        coordinate[destination][0],
                        coordinate[destination][1],
                        coordinate[destination][2] + count[destination],
                    ),
                    -90,
                    -90,
                    0,
                    1000,
                )
                time.sleep(result[2] / 1000)
                print(result)
                if not result:
                    continue

                # if not __isRunning:
                #     continue
                # 旋转角度放下
                servo2_angle = getAngle(
                    coordinate[destination][0], coordinate[destination][1], -90
                )
                Board.setBusServoPulse(2, servo2_angle, 300)
                time.sleep(0.3)

                # if not __isRunning:
                #     continue
                Board.setBusServoPulse(1, servo1 - 200, 300)  # 爪子张开  ，放下物体
                time.sleep(0.3)
                put_it = True
                n = count[destination]
                count[destination] = n + 1
                print("num %s" % destination + "=%d" % (n + 1))

                # if not __isRunning:
                #     continue
                result = AK.setPitchRangeMoving(
                    (coordinate[destination][0], coordinate[destination][1], 12),
                    -90,
                    -90,
                    0,
                    500,
                )
                time.sleep(result[2] / 1000)
                destination = None
                cur_orderid = None
                initMove()  # 回到初始位置
            else:
                put_it = False
                if _stop:
                    _stop = False
                    Board.setBusServoPulse(1, servo1 - 70, 300)
                    time.sleep(0.3)
                    Board.setBusServoPulse(2, 500, 500)
                    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                    # initMove()
                time.sleep(0.01)
            endTime = time.perf_counter()
            print(f"开始抓取到完成用时:{(endTime-startTime)*1000}ms")
            res = rpcclient.call("Update_state", ArmPi_id)
            print(res)

th = threading.Thread(target=move)
th.setDaemon(True)  # 守护线程
th.start()

def angle(a, R=10):
    x = 0.0
    y = 0.0
    if a >= 0 and a <= 180:
        if a == 0:
            x = R
            y = 0.0
            return x, y
        elif a == 90:
            x = 0
            y = R
            return x, y
        elif a == 180:
            x = -R
            y = 0.0
            return x, y
        else:
            y = sin(pi / 180 * a) * R
            x = y / tan(pi / 180 * a)
            return x, y
    else:
        pass


def decodeAllQR(image):
    """识别当前画面所有二维码，存储在orderblocks，订单号存储在orderIDs
    返回最大二维码方块信息
    """
    global orderblocks
    global orderIDs
    barcodes = pyzbar.decode(image)
    num = len(barcodes)
    closest_block=None
    min_Y=100
    if not barcodes:
        print("No barcode found.")
        time.sleep(1)
        return image,closest_block
    for barcode in barcodes:
        # 计算二维码轮廓面积
        # area = barcode.rect.width * barcode.rect.height
        # # 如果当前二维码面积更大，则记录下该二维码
        # if area > max_area:
        #     max_area = area
        #     max_barcode = barcode

        orderID = barcode.data.decode("utf-8")
        orderIDs.add(orderID)
        # 找到二维码的最小边框位置
        rect = cv2.minAreaRect(np.array(barcode.polygon, np.int32))
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        #获取方块对应的中心x、y坐标
        if rect is not None:
            if box is not None:
                # 获取方块的现实世界坐标
                roi = getROI(box)  # 获取极值点
                img_centerx, img_centery = getCenter(
                    rect, roi, size, square_length
                )  # 获取木块中心坐标
                rotation_angle = rect[2]
                world_x, world_y = convertCoordinate(
                    img_centerx, img_centery, size
                )  # 转换为现实世界坐标
                print(f"orderid={orderID} x={world_x} y={world_y}")
                insert_order(orderID,(world_x, world_y),rotation_angle)
                if world_y<min_Y:
                    min_Y=world_y
                    closest_block=[orderID,(world_x, world_y),rotation_angle]
    
    return image, closest_block

def get_orderblock_fromSet():
    """ 
    遍历字典找到最近的方块
    """
    global orderblocks

    res_orderid=None
    res_x=None
    res_y=100
    res_angle=None
    if len(orderblocks):
        for orderid,(x_y,angle) in orderblocks.items():
            if x_y[1]<res_y:
                res_orderid=orderid
                res_x=x_y[0]
                res_y=x_y[1]
                res_angle=angle
    closest_block=[res_orderid,(res_x, res_y),res_angle]
    return closest_block

def set_order_adress(order_adress):
    global cur_orderid
    global destination
    global orderblocks
    global world_X
    global world_Y
    global rotation_angle

    cur_orderid=order_adress[0]
    order_adress=order_adress[1]
    for orderid,(x_y,angle) in orderblocks.items():
            if orderid==cur_orderid:
                world_X=x_y[0]
                world_Y=x_y[1]
                rotation_angle=angle

    while cur_orderid!=None:
        time.sleep(0.01)
        continue

    return (True,True)


def run(frame):
    global _stop
    global get_roi
    global unreachable
    global destination
    global count
    global rpcclient
    global cur_orderid
    global rotation_angle
    global world_X, world_Y
    global ArmPi_id

    cur_orderid = None
    destination=None
    initMove()
    time.sleep(1)

    if frame is not None:
        cv2.imshow('frame', frame)
    else:
        print("no frame")
        if cur_orderid==None and destination==None:
            return 
    #识别所有二维码和地址信息
    print("开始扫描所有二维码")
    if len(orderblocks):
        cur_orderBlock=get_orderblock_fromSet()
        if cur_orderBlock[0]==None:
            time.sleep(1)
    else:
        # 检测图像中最近的二维码
        img, cur_orderBlock = decodeAllQR(frame)
        if cur_orderBlock==None:
            time.sleep(1)
    cv2.imshow("img", img)
    print("over*******************")

    return frame

if __name__ == "__main__":
    # 运行子线程
    initMove()
    run()
    
