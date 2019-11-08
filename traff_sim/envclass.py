from typing import List
import numpy as np
import scipy.stats as st


# 【保证getLink和getDirection可以使用】
class Car(object):

    def __init__(self, velocity=None, origin=None, destination=None, position=None, direction=None, startTime=None,
                 firstIntersect=None, endIntersect=None, antecedeCar=None, link=None):
        self.__velocity = velocity
        self.__origin: Origin = origin  # origin 是Origin对象，不是普通变量
        self.__destination = destination.copy()  # [x,y] 终点位置
        self.__position = position.copy()  # list: [x,y] 当前位置
        self.__startTime = startTime  # 进入网络的时间，单位是s ，初始时是0
        self.__endTime = None
        self.__waitTurn = 0
        self.__antecedeCar = antecedeCar  # 上一辆车
        self.detour = 0
        # self.__nextCar = None

        self.__link: Link = link  # 当前link
        self.__direction = direction.copy()  # 当前方向
        self.__firstIntersection = firstIntersect  # 只在放入路网时候使用
        self.__endIntersection = endIntersect  # 最后一个路口
        self.link_to_go = []  # 下一个link
        self.direction_to_go = []  # 下一个方向
        self.intersection_to_go = []  # 当前intersect + 下一个intersect
        self.wait_done = False
        self.re_routed = False
        self.direction_toTurn: List[np.ndarray] = []  # 未来要走的转向
        self.__nextLink: Link = None  # 下一个link：只用一次
        self.turn_record = []  # 走过的记录
        self.link_record = []  # 走过的记录

    def destroy(self):
        self.__origin = None
        self.__destination = None
        self.__position = None
        self.__direction = None
        # self.direction_toTurn.clear()
        self.__antecedeCar = None
        # self.__nextCar = None
        # self.__nextLink = None
        self.__link = None
        self.__endTime = None
        self.__firstIntersection = None
        self.detour = 0

    def getFirstIntersection(self):
        return self.__firstIntersection

    def setFirstIntersection(self, intersect):
        self.__firstIntersection = intersect

    def getEndIntersection(self):
        return self.__endIntersection

    def setEndIntersection(self, intersect):
        self.__endIntersection = intersect

    def getVelocity(self):
        return self.__velocity

    def setVelocity(self, velocity):
        self.__velocity = velocity

    def getOriginal(self):
        return self.__origin

    def setOriginal(self, origin):
        self.__origin = origin

    def getDestination(self) -> np.ndarray:
        return self.__destination.copy()

    def setDestination(self, destination):
        self.__destination = destination.copy()

    def getPosition(self):
        return self.__position.copy()

    def setPosition(self, position):
        self.__position = position.copy()

    def getStartTime(self):
        return self.__startTime

    def setStartTime(self, startTime):
        self.__startTime = startTime

    def getEndTime(self):
        return self.__endTime

    def setEndTime(self, endTime):
        self.__endTime = endTime

    def getDirection(self):
        return self.__direction.copy()

    def setDirection(self, direction):
        self.__direction = direction.copy()

    def getNextLink(self):  # Link对象
        return self.__nextLink

    def setNextLink(self, nextLink):
        self.__nextLink = nextLink

    def getAntecedeCar(self):
        return self.__antecedeCar

    def setAntecedeCar(self, antecedeCar):
        self.__antecedeCar = antecedeCar

    def getWaitTurn(self):
        return self.__waitTurn

    def setWaitTurn(self, nextTurn):
        self.__waitTurn = nextTurn

    def getLink(self):
        return self.__link

    def setLink(self, link):
        self.__link = link

    def __str__(self) -> str:
        return "o:{},d:{},dir:{},v:{},position:{},super:{}.".format(
            self.__origin.getIndex(),
            self.__destination,
            self.__direction,
            self.__velocity,
            self.__position,
            super().__str__()
        )


class Intersection(object):

    def __init__(self, position, index):
        self.__position: np.ndarray = position.copy()  # np.ndarray [x,y]
        self.__index = index
        self.__link_N: Link = None
        self.__link_S: Link = None
        self.__link_W: Link = None
        self.__link_E: Link = None
        self.__hopMap: HopMap = None
        self.destinations: List[Intersection] = []  # 到达各个终点
        self.splitRates: List[SplitRate] = []  # 到达各个终点的路由
        self.split_rate_map = {}  # 到达各个终点的路由
        self.go_N_num = 0  # 博弈相关
        self.go_S_num = 0  # 博弈相关
        self.go_W_num = 0  # 博弈相关
        self.go_E_num = 0  # 博弈相关

    def destroy(self):
        self.__position = None
        self.__link_N = None
        self.__link_S = None
        self.__link_W = None
        self.__link_E = None
        self.__hopMap = None
        self.destinations = None
        self.splitRates = None

    def getPosition(self) -> np.ndarray:
        return self.__position.copy()

    def getIndex(self):
        return self.__index

    def getLink_N(self):
        return self.__link_N

    def setLink_N(self, link_N):
        self.__link_N = link_N

    def getLink_S(self):
        return self.__link_S

    def setLink_S(self, link_S):
        self.__link_S = link_S

    def getLink_W(self):
        return self.__link_W

    def setLink_W(self, link_W):
        self.__link_W = link_W

    def getLink_E(self):
        return self.__link_E

    def setLink_E(self, link_E):
        self.__link_E = link_E

    def getLink_E(self):
        return self.__link_E

    def setLink_E(self, link_E):
        self.__link_E = link_E

    def getHopMap(self):
        return self.__hopMap

    def setHopMap(self, hopMap):
        self.__hopMap = hopMap

    def __str__(self) -> str:
        ret = "index:{},position:{},des num:{},split num:{},".format(
            self.__index,
            self.__position,
            len(self.destinations),
            len(self.splitRates)
        )
        return ret + super().__str__()


class Link(object):

    def __init__(self, length, position1, position2, index):
        self.__length = length
        self.__position1 = position1.copy()  # [x1,y1]
        self.__position2 = position2.copy()  # [x2,y2]
        self.node1_id = -1
        self.node2_id = -1
        self.__index = index
        self.carOnRoad_positive: List[Car] = []
        self.carOnRoad_negative: List[Car] = []
        self.carWait_positive: List[Car] = []
        self.carWait_negative: List[Car] = []
        self.__intersection_N: Intersection = None
        self.__intersection_S: Intersection = None
        self.__intersection_W: Intersection = None
        self.__intersection_E: Intersection = None
        self.travelTimes_predict: List[np.ndarray] = []  # positive,negative
        self.car_positive = 0  # 正向n-1辆车的数量
        self.car_negative = 0  # 反向n-1辆车的数量
        self.carNum = 0  # 道路上车辆数，初始0
        self.waitedCarNum = 0  # 等待进入道路车辆数 初始0
        self.effectedCarNum = None  # 受影响的车辆数目

        self.decay = 1.0
        self.in_guid = False

    def destroy(self):
        self.__position1 = None
        self.__position2 = None
        self.carOnRoad_positive = None
        self.carOnRoad_negative = None
        self.carWait_positive = None
        self.carWait_negative = None
        self.__intersection_N = None
        self.__intersection_S = None
        self.__intersection_W = None
        self.__intersection_E = None
        self.travelTimes_predict.clear()

    def getLength(self):
        return self.__length

    def getPosition1(self):
        return self.__position1.copy()

    def getPosition2(self):
        return self.__position2.copy()

    def getIndex(self):
        return self.__index

    def getIntersection_N(self):
        return self.__intersection_N

    def setIntersection_N(self, intersection_N):
        self.__intersection_N = intersection_N

    def getIntersection_S(self):
        return self.__intersection_S

    def setIntersection_S(self, intersection_S):
        self.__intersection_S = intersection_S

    def getIntersection_W(self):
        return self.__intersection_W

    def setIntersection_W(self, intersection_W):
        self.__intersection_W = intersection_W

    def getIntersection_E(self):
        return self.__intersection_E

    def setIntersection_E(self, intersection_E):
        self.__intersection_E = intersection_E

    def __str__(self) -> str:
        node1, node2 = None, None
        if self.__intersection_E:
            node1 = self.__intersection_E
        if self.__intersection_W:
            node2 = self.__intersection_W
        if self.__intersection_N:
            node1 = self.__intersection_N
        if self.__intersection_S:
            node2 = self.__intersection_S

        # ret = "index:{},pos1:{},pos2:{},length:{},node1:{},node2:{}," \
        #       "car_num:{},car_pos:{},car_neg:{},car_wait{}".format(
        ret = "{},{},{},{},{},{},{},{},{},{}".format(
            self.__index,
            self.__position1,
            self.__position2,
            self.__length,
            self.node1_id,
            self.node2_id,
            self.carNum,
            self.car_positive,
            self.car_negative,
            self.waitedCarNum
        )
        return ret + super().__str__()


class Origin(object):

    def __init__(self, index, mean, position, time_num=60):  # 根据每分钟的平均车辆数mean，和一共仿真的分钟数time，初始化该class
        self.__carNum = st.poisson.rvs(mean, size=time_num)  # 是个列表
        self.__position = position.copy()  # postion :nparray = [x,y]
        self.cars = []
        self.__direction = None  # nparray = [x,y]
        self.__index = index
        self.nextInstesect = None  # 有时候有一个，有时候有两个
        self.leave_car_index = 0  # 每个ori已经走了多少辆车了

    def destroy(self):
        self.__position = None
        self.__carNum = None
        self.__direction = None

    def setIndex(self, index):
        self.__index = index

    def getIndex(self):
        return self.__index

    def getCarNum(self):
        return self.__carNum.copy()

    def getPosition(self):
        return self.__position.copy()

    def setDirection(self, direction):
        self.__direction = direction.copy()

    def getDirection(self):
        return self.__direction.copy()

    def __str__(self) -> str:
        ret = "index:{},car_num:{},position:{},direction:{}".format(
            self.__index,
            self.__carNum,
            self.__position,
            self.__direction)
        return ret + super().__str__()


class HopMap(object):

    def __init__(self):
        self.intersections_oneHop: List[Intersection] = []
        self.links: List[Link] = []
        self.turns: List[np.ndarray] = []

    def destroy(self):
        self.intersections_oneHop.clear()
        self.links.clear()
        self.turns.clear()


class Path(object):

    def __init__(self, origin, destination):  # 根据每分钟的平均车辆数mean，和一共仿真的分钟数time，初始化该class
        self.__origin: Intersection = origin
        self.__destination: Intersection = destination
        self.links: List[Link] = []  # List 的List，内部的List是link的顺序, 外部表示一共有6条不同的path，对于一个OD
        self.turns: List[np.ndarray] = []
        self.turn_count: int = -1
        self.distance: int = -1
        self.distance_ratio: int = -1
        self.intersects: List[Intersection] = [origin]

    def destroy(self):
        del self.__origin
        del self.__destination
        self.links.clear()
        del self.links
        self.turns.clear()
        del self.turns
        self.intersects.clear()
        del self.intersects

    def setOrigin(self, origin):
        self.__origin = origin

    def getOrigin(self):
        return self.__origin

    def setDestination(self, destination):
        self.__destination = destination

    def getDestination(self):
        return self.__destination

    def __str__(self) -> str:
        ret1 = [str(intersect.getIndex()) for intersect in self.intersects]
        ret2 = ["({},{})".format(turn[0], turn[1]) for turn in self.turns]
        ret3 = ["[({},{}),({},{})]".format(link.getPosition1()[0], link.getPosition1()[1],
                                           link.getPosition2()[0], link.getPosition2()[1]) for link in self.links]
        return "dis:{},turn_count:{},intersections:{},turns:{},links:{}.".format(
            str(self.distance), str(self.turn_count),
            "->".join(ret1),
            "->".join(ret2),
            "->".join(ret3)
        )

        # return super().__str__()


class SplitRate(object):

    def __init__(self, origin, destination):
        self.__origin: Intersection = origin
        self.__destination: Intersection = destination
        self.paths: List[Path] = []
        self.__splitting_rate: np.ndarray = None  # 一个OD对的splitting rate
        self.base_distance = -1

    def getOrigin(self):
        return self.__origin

    def getDestination(self):
        return self.__destination

    def getSplittingRate(self):
        return self.__splitting_rate.copy()

    def setSplittingRate(self, splittingRate):
        self.__splitting_rate = splittingRate.copy()

    def __str__(self) -> str:
        ret = "{},{},{},[{}],".format(
            self.__origin.getIndex(), self.__destination.getIndex(), len(self.paths),
            ",".join(["%.3f" % num for num in self.__splitting_rate])
        )
        return ret + super().__str__()
