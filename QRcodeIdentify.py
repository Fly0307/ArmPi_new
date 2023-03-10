import cv2
import time
import Camera
import threading
import sys
sys.path.append('/home/pi/ArmPi/')
# sys.path.append('/home/pi/.local/lib/python3.7/site-packages')
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from math import *
import pyzbar.pyzbar as pyzbar


if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

# 调用机械臂控制函数
AK = ArmIK()

range_rgb = {
    '成都':   (0, 0, 255),
    '上海':  (255, 0, 0),
    '天津': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}
# 放置位置,代表地址，和demo图纸不同颜色对应
# placement_area = {
#     '成都':   (-15 + 0.5, 12 - 0.5, 1.5),
#     '天津': (-15 + 0.5, 6 - 0.5,  1.5),
#     '上海':  (-15 + 0.5, 0 - 0.5,  1.5),
# }
# coordinate = {
#     '成都':   (-15 + 0.5, 12 - 0.5, 2),
#     '天津': (-15 + 0.5, 6 - 0.5,  2),
#     '上海':  (-15 + 0.5, 0 - 0.5,  2),
# }
#坐标以机械臂底座舵机为中心
coordinate = {
        '成都':   (-15 + 0.5, 6 - 0.5, 2),
        '天津': (-15 + 0.5, -0 - 0.5,  2),
        '上海':  (-15 + 0.5, -6 - 0.5,  2),
    }


# 夹持器夹取时闭合的角度
servo1 = 500
# 初始位置


def initMove():
    print('QR initmove()')
    Board.setBusServoPulse(1, servo1 - 50, 300)
    Board.setBusServoPulse(2, 500, 500)
    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90,1000)

# initMove()

def setBuzzer(timer):
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)

# 设置扩展板的RGB灯颜色使其跟要追踪的颜色一致


def set_rgb(color):
    if color == "red":
        Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
        Board.RGB.show()
    elif color == "green":
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
        Board.RGB.show()
    elif color == "blue":
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
        Board.RGB.show()
    else:
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
        Board.RGB.show()


count = {'成都':   0,
    '天津': 0,
    '上海':  0,}  # 放置高度计数
_stop = False
get_roi = False
__isRunning = False
detect_block = 'None'
start_pick_up = False  # 为true时抓取方块
pick_up=False
start_pick_down = False  # 为true时放置方块，均为false时恢复初始状态
start_count_t1 = True
num=0

size = (640, 480)  # 设置采集图片(流)大小
rotation_angle = 0
unreachable = False  # 判断是否能够抓取
reachtime = 0  # 抵达指定位置的时间
catchtime = 0  # 抓取的时间
text='null'
state=0
last_text='null'
world_X, world_Y = 0, 0

def reset():
    global _stop
    global count
    global get_roi
    global detect_block
    global start_pick_up
    global start_pick_down
    global pick_up
    global z_r, z_g, z_b, z

    count = count = {'成都':   0,
    '天津': 0,
    '上海':  0,} 
    _stop = False
    get_roi = False
    detect_block = 'None'
    start_pick_up = False
    pick_up=False
    start_pick_down = False
    z_r = coordinate['成都'][2]
    z_g = coordinate['天津'][2]
    z_b = coordinate['上海'][2]
    z = z_r


def init():
    print("QRSorting Init")
    initMove()
    print("Init finished")

def start():
    global __isRunning
    reset()
    __isRunning = True
    print("QRSorting Start")


def stop():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("QRSorting Stop")


def exit():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("QRSorting Exit")




# 控制机械臂移动
def move():
    global _stop
    global get_roi
    global move_square
    global __isRunning
    global unreachable
    global detect_block
    global count
    global text
    global start_pick_up
    global start_pick_down
    global pick_up
    global rotation_angle
    global world_X, world_Y
    global z_r, z_g, z_b, z

    while True:
        if __isRunning:
            startTime=0
            print("detect_block=%s" %(detect_block))
            get_it=False
            count_num=0
            while not get_it:
                startTime=time.perf_counter()
                if detect_block != 'None' and start_pick_up:  
                    # 如果抓取2次后还未抓取成功，则清空识别记录重新识别
                    if count_num>1:
                        text='null'
                        start_pick_up=False
                        start_pick_down=False
                        detect_block = 'None'
                        initMove()
                        break
                    set_rgb(detect_block)
                    # setBuzzer(0.1)
                    # reachtime = 0.0
                    # 高度累加
                    print(detect_block)

                    print("move to world_X=%d"%(world_X) +"and world_Y=%d"%(world_Y))
                    result = AK.setPitchRangeMoving(
                        (world_X, world_Y, 7), -90, -90, 0,1500)  # 移到目标位置上方，高度5cm
                    time.sleep(result[2]/1000)
                    if result == False:
                        unreachable = True
                    else:
                        unreachable = False
                        if not __isRunning:
                            continue
                        # 计算夹持器需要旋转的角度
                        servo2_angle = getAngle(world_X, world_Y, rotation_angle)
                        Board.setBusServoPulse(2, servo2_angle, 300)  # 旋转爪子
                        time.sleep(0.3)

                        if not __isRunning:
                            continue
                        result=AK.setPitchRangeMoving(
                            (world_X, world_Y, 5), -90, -90, 0,1000)  # 降低高度到4cm
                        time.sleep(result[2]/1000)
                        if result == False:
                            print("can't reach")
                        Board.setBusServoPulse(1, servo1 - 280, 300)  # 爪子张开
                        time.sleep(0.3)
                        if not __isRunning:
                            continue
                        result=AK.setPitchRangeMoving(
                            (world_X, world_Y, 1.5), -90, -90, 0,500)  # 降低高度到2cm
                        time.sleep(result[2]/1000)

                        if not __isRunning:
                            continue
                        Board.setBusServoPulse(1, servo1, 300)  # 夹持器闭合
                        time.sleep(0.3)

                        if not __isRunning:
                            continue
                        Board.setBusServoPulse(2, 500, 300)
                        result=AK.setPitchRangeMoving(
                            (world_X, world_Y, 9), -90, -90, 0,1000)  # 机械臂抬起
                        time.sleep(result[2]/1000)
                        servo1_now=Board.getBusServoPulse(1)
                        # print("servo1_now=%d"%servo1_now)
                        #未夹取成功
                        if servo1_now>= 490:
                            print("don't get it")
                            count_num+=1
                            if text=="null":
                                start_pick_up=False
                                start_pick_down=False
                                detect_block = 'None'
                                initMove()
                            continue
                        else: 
                            get_it=True
                        result=AK.setPitchRangeMoving(
                            (0, 10, 12), -90, -90, 0,1000)  # 机械臂抬起
                        time.sleep(result[2]/1000)
                        pick_up=True
                        start_pick_up=False
                        print("机械臂抬起")
                else:
                    get_it=False
            if detect_block=='None':
                time.sleep(1)
                continue
            put_it=False
            while (not put_it and text!='null'):
                if detect_block!='None'and start_pick_down:
                    print("机械臂开始放下 detect_block=%s"%(detect_block))
                    pick_up=False
                    start_pick_up=False
                    if not __isRunning:
                        continue
                    print(detect_block)
                    result=AK.setPitchRangeMoving(
                        (coordinate[detect_block][0], coordinate[detect_block][1], 12), -90, -90, 0,1000)
                    time.sleep(result[2]/1000)
                    print(result)

                    if not __isRunning:
                        continue
                    result=AK.setPitchRangeMoving(
                        (coordinate[detect_block][0], coordinate[detect_block][1], coordinate[detect_block][2]+count[detect_block]), -90, -90, 0,1000)
                    time.sleep(result[2]/1000)
                    print(result)
                    if not result:
                        continue

                    if not __isRunning:
                        continue
                    #旋转角度放下
                    servo2_angle = getAngle(coordinate[detect_block][0], coordinate[detect_block][1], -90)
                    Board.setBusServoPulse(2, servo2_angle, 300)
                    time.sleep(0.3)

                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(1, servo1 - 200, 300)  # 爪子张开  ，放下物体
                    time.sleep(0.3)
                    put_it=True
                    n=count[detect_block]
                    count[detect_block]=n+1
                    print("num %s"%(detect_block)+"=%d"%(n+1))

                    if not __isRunning:
                        continue
                    result=AK.setPitchRangeMoving(
                        (coordinate[detect_block][0], coordinate[detect_block][1], 12), -90, -90, 0,500)
                    time.sleep(result[2]/1000)
                    detect_block = 'None'
                    get_roi = False
                    start_pick_up = False
                    start_pick_down=False
                    text='null'
                    set_rgb(detect_block)
                    initMove()  # 回到初始位置
                else:
                    put_it=False
                    if _stop:
                        _stop = False
                        Board.setBusServoPulse(1, servo1 - 70, 300)
                        time.sleep(0.3)
                        Board.setBusServoPulse(2, 500, 500)
                        AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                        # initMove()
                    time.sleep(0.01)
                endTime=time.perf_counter()
                print(f"开始抓取到完成用时:{(endTime-startTime)*1000}ms")


# 运行子线程
th = threading.Thread(target=move)
th.setDaemon(True)#守护线程
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
            y = sin(pi/180*a)*R
            x = y/tan(pi/180*a)
            return x, y
    else:
        pass


def decodeDisplay(image):
    """ 
    识别当前所有二维码，返回最大二维码的值
    """
    global last_text
    barcodes = pyzbar.decode(image)
    num=len(barcodes)
    data = []
    box=None
    rect=None
    startTime=time.perf_counter()
    if not barcodes:
        print('No barcode found.')
        time.sleep(1)
        return image,box,rect, data
    # 记录最大面积的二维码
    max_area = 0
    max_barcode = None
    # 遍历所有二维码
    for barcode in barcodes:
        # 计算二维码轮廓面积
        area = barcode.rect.width * barcode.rect.height
        # 如果当前二维码面积更大，则记录下该二维码
        if area > max_area:
            max_area = area
            max_barcode = barcode
    # 如果找到了最大面积的二维码，则输出二维码信息和边框位置
    if max_barcode is not None:
        # 输出二维码信息
        # 找到二维码的最小边框位置
        max_rect = cv2.minAreaRect(np.array(max_barcode.polygon, np.int32))
        box = cv2.boxPoints(max_rect)
        box = np.int0(box)
    (x, y, w, h) = max_barcode.rect
    barcodeData = max_barcode.data.decode("utf-8")
    data.append([x, y, w, h, barcodeData])
    endTime=time.perf_counter()
    print(f'识别二维码时间为:{(endTime-startTime)*1000}ms')
    # orderIDs=[]
    # for barcode in barcodes:
    #     orderID = barcode.data.decode("utf-8")
    #     (x, y, w, h) = barcode.rect
    #     orderIDs.append([orderID,x,y])
    # startTime=time.perf_counter()
    # print(f'识别画面所有二维码时间为:{(startTime-endTime)*1000}ms')
    # print(orderIDs)
    return image,box,max_rect, data


# target_color,暂时用颜色代替地址
#心跳检测，用以返回当前机械臂状态
# 0 未识别到数据  1抓取阶段
# 2 放置阶段  3抓取悬空阶段
def Heartbeat(alive):
    global start_pick_up
    global start_pick_down
    global pick_up
    global text
    global state
    # print('func Heartbeat()',alive)
    if text=='null':
        state=0
        return (True,(0))
    elif alive:
        start_pick_up=True
        state=1
        return (True,(1))
    elif pick_up:
        state=3
        return (True,(3))
    elif start_pick_up:
        #抓取阶段
        state=1
        return (True,(1))
    elif start_pick_down:
        #抓取结束放置阶段
        state=2
        return (True,(2))
    else:
        #没有抓也没有放，但识别到
        state=4
        return (True,(4))

def setTarget(target_color):
    global detect_block
    global text
    global last_text
    global start_pick_up
    global start_pick_down
    print("func setTarget() started target_color=%s"%(target_color)+" text=%s"%(text))
    # RPC GetOrderId()方法
    if target_color=='double':
        #重复订单,重新识别
        last_text=text
        detect_block='None'
        start_pick_up=False
        start_pick_down=False
        return (True,(text))
    if target_color=='None':
        return (True,(text))
    else:
        print("COLOR", target_color)
        if text!='null':
            start_pick_down=True
            if type(target_color) == tuple:
                detect_block=target_color[0]
            else:
                detect_block=target_color
            print("detect_block", detect_block)
        else:
            start_pick_down=False
        return (True, (text))
    
def QRcode_sort_debug():
    print('func QRcode_sort() started')
    global detect_block
    global rotation_angle
    global start_pick_up
    global start_pick_down
    global text
    global get_roi
    global world_X, world_Y

    # init()
    start()
    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        frame = my_camera.frame
        if frame is not None:
            img = frame.copy()
            # 检测图像中的二维码内容,仅限一个
            if text=='null':
                img,box, rect,data = decodeDisplay(img)
            else:
                img=img
                box=None
                rect=None
            # 计算出二维码的位置和盒子位置
            # box, rect, black_box = detect(img)
            # print(rect)
            if rect is not None:
                if box is not None:
                # 获取方块的现实世界坐标
                    roi = getROI(box)  # 获取roi区域
                    get_roi = True
                    img_centerx, img_centery = getCenter(
                                rect, roi, size, square_length)  # 获取木块中心坐标
                    rotation_angle=rect[2]
                    world_X, world_Y = convertCoordinate(
                                img_centerx, img_centery, size)  # 转换为现实世界坐标
                    # print("world_X= %d" % (world_X)+" world_Y=%d" % (world_Y))
                    # 框出二维码或二维码部分
                    # cv2.drawContours(img, [box], -1, (0, 255, 0), 2)
                    cv2.imshow('img', img)
                    if len(data)==0:
                        # text = data[0][4]
                        # start_pick_up = True
                        print('return text')
                        # return (True,(text))
                    else:
                        # print('return None')
                        # return (True,('None'))                    
                        if len(data) != 0:
                            # 在frame上显示识别内容
                            text = data[0][4]
                            cv2.putText(frame, text, (data[0][0], data[0][1] - 10), cv2.FONT_HERSHEY_SIMPLEX,.5, (0, 0, 125), 2)
                            # xx, yy = convertCoordinate(
                            #     data[0][0], data[0][1], size)
                            # print(xx, yy)
                            # 检测到特定二维码内容时才会抓取
                            if data[0][4] == '000000004' or data[0][4]=='000000019':
                                detect_block = '上海'
                                start_pick_up = True
                                start_pick_down=True
                                # coordinate['上海'] = (xx+2, yy+5, 12)
                            elif data[0][4] == '100000020' or data[0][4]=='000000009':
                                detect_block = '成都'
                                start_pick_up = True
                                start_pick_down=True
                                # coordinate['成都'] = (xx+2, yy+5, 12)
                            elif data[0][4] == '000000003'or data[0][4]=='000000005':
                                detect_block = '天津'
                                start_pick_up = True
                                start_pick_down=True
                                # coordinate['天津'] = (xx+2, yy+5, 12)
                            else:
                                detect_block = 'None'
                            # return text
                        
            else:
                if start_pick_up and not start_pick_down:
                    #画面中无二维码且未抓取到
                    text='null'
                    start_pick_up=False
                    start_pick_down=False
            # img = run(img)
            cv2.imshow('frame', frame)
            frame=None
            key = cv2.waitKey(1)
            if key == 27:
                break
            else:
                if not __isRunning:
                    my_camera.camera_close()
                    cv2.destroyAllWindows()
                    return
                # 返回编号
                # 如果采用预抓取则不需要返回方块位置
                # return 'None'

def run(frame):
    global _stop
    global get_roi
    global move_square
    global __isRunning
    global unreachable
    global detect_block
    global count
    global text
    global state
    global last_text
    global start_pick_up
    global rotation_angle
    global world_X, world_Y
    global z_r, z_g, z_b, z
    print('func run() started text=%s'%(text)+' state=%d'%(state))
    if frame is not None:
        img=frame.copy()
        cv2.imshow('img', frame)
    else:
        print("no frame")
        return
    if text!='null':
        return img
    # 检测图像中的二维码内容,仅限一个
    img,box, rect,data = decodeDisplay(img)
    # 计算出二维码的位置和盒子位置
    if rect is not None:
        if box is not None:
        # 获取方块的现实世界坐标
            roi = getROI(box)  # 获取roi区域
            get_roi = True
            img_centerx, img_centery = getCenter(
                        rect, roi, size, square_length)  # 获取木块中心坐标
            rotation_angle=rect[2]
            world_X, world_Y = convertCoordinate(
                        img_centerx, img_centery, size)  # 转换为现实世界坐标
            print("world_X= %d" % (world_X)+" world_Y=%d" % (world_Y))
            
            if len(data) != 0 :
                text = data[0][4]
            state=4
    cv2.imshow('frame', frame)
    return img

if __name__ == '__main__':
    print('func main() started')
    QRcode_sort_debug()