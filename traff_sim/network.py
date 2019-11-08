from traff_sim.constant import constant
from traff_sim.envclass import *
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from random import choice, random, randint


def random_destination(mainDestination, others, probability):
    d = random()  # 随机生成目的地
    if d < probability:  # 分给对角
        d = choice(mainDestination)
    else:
        d = choice(others)
    return d


def random_path(data, weight) -> Path:
    weight = weight * 10000
    weight = weight.astype(np.int32)
    total = sum(weight)
    rad = randint(1, total)

    cur_total = 0
    res = None
    for k, v in zip(data, weight):
        cur_total += v
        if rad <= cur_total:
            res = k
            break
    return res


def random_direction(direction1, direction2):
    if randint(0, 1):
        return direction1
    else:
        return direction2


def get_random_direction(difference) -> np.ndarray:
    if difference[0] == 0:
        if difference[1] > 0:
            direction_new = np.array([0, 1])
        else:
            direction_new = np.array([0, -1])
    elif difference[0] > 0:
        if difference[1] == 0:
            direction_new = np.array([1, 0])
        elif difference[1] > 0:
            if abs(difference[0]) == constant.length_edg:
                direction_new = np.array([0, 1])
            elif abs(difference[1]) == constant.length_edg:
                direction_new = np.array([1, 0])
            else:
                direction_new = random_direction(np.array([1, 0]), np.array([0, 1]))
        else:
            if abs(difference[0]) == constant.length_edg:
                direction_new = np.array([0, -1])
            elif abs(difference[1]) == constant.length_edg:
                direction_new = np.array([1, 0])
            else:
                direction_new = random_direction(np.array([1, 0]), np.array([0, -1]))
    else:
        if difference[1] == 0:
            direction_new = np.array([-1, 0])
        elif difference[1] > 0:
            if abs(difference[0]) == constant.length_edg:
                direction_new = np.array([0, 1])
            elif abs(difference[1]) == constant.length_edg:
                direction_new = np.array([-1, 0])
            else:
                direction_new = random_direction(np.array([-1, 0]), np.array([0, 1]))
        else:
            if abs(difference[0]) == constant.length_edg:
                direction_new = np.array([0, -1])
            elif abs(difference[1]) == constant.length_edg:
                direction_new = np.array([-1, 0])
            else:
                direction_new = random_direction(np.array([-1, 0]), np.array([0, -1]))
    return direction_new


def get_velocity_num(num_car, capacity, maxCapacity):
    if num_car <= maxCapacity:
        v = constant.MAX_VELOCITY / (1 + constant.ALPH * pow(num_car / capacity, constant.BETA))
    else:
        v = constant.MAX_VELOCITY / (1 + constant.ALPH * pow(maxCapacity / capacity, constant.BETA))
    return v


# 得到转向的不同时间
def get_time_turn(oldDirection: np.array([]), newDirection: np.array([])):
    if (oldDirection == newDirection).all():
        delay = constant.delay_turn_straight  # straight
    elif (oldDirection == newDirection * (-1)).all():
        delay = constant.delay_turn_u
    else:
        diff = newDirection - oldDirection
        if (oldDirection[0] + oldDirection[1]) * diff[0] < 0:
            delay = constant.delay_turn_left
        else:
            delay = constant.delay_turn_right
    return delay


class Network:

    def __init__(self, init=True):

        if init:
            # 0、创建路口、创建车辆列表
            intersections: List[Intersection] = []
            for index, position_1 in enumerate(constant.position_intersection):
                intersect = Intersection(position_1, index)
                index += 1
                intersections.append(intersect)

            cars: List[Car] = []
            cars_append = cars.append

            origins: List[Origin] = []
            for i in range(0, len(constant.position_origin)):
                origin = Origin(i, constant.o_car_num[i], constant.position_origin[i],
                                int(constant.time_simulate / 60))
                if i < 2:
                    origin.setDirection(np.array([0, 1]))
                elif i < 4 or i == 8:
                    origin.setDirection(np.array([1, 0]))
                elif i < 6 or i == 9:
                    origin.setDirection(np.array([-1, 0]))
                elif i < 8:
                    origin.setDirection(np.array([0, -1]))

                if i in [1, 4]:
                    origin.nextInstesect = intersections[13]
                elif i in [3, 6]:
                    origin.nextInstesect = intersections[21]
                elif i in [5, 7]:
                    origin.nextInstesect = intersections[17]
                elif i in [0, 2]:
                    origin.nextInstesect = intersections[9]
                elif i == 8:
                    origin.nextInstesect = [intersections[23], intersections[2]]
                else:  # i == 9
                    origin.nextInstesect = [intersections[1], intersections[15]]
                origins.append(origin)
                # print(origin)

            for i in range(0, len(origins)):
                candidate_destination = [d for d in range(0, len(constant.position_intersection))]
                star_time: int = 0
                for carNum in origins[i].getCarNum():
                    if carNum <= 0: continue
                    interval = 60 / carNum
                    for j in range(0, carNum):
                        if i == 1 or i == 4:
                            d = random_destination([21], candidate_destination, 0.8)
                            direction = origins[i].getDirection()
                            nextIntersect = intersections[13]
                        elif i == 3 or i == 6:
                            d = random_destination([13], candidate_destination, 0.8)
                            direction = origins[i].getDirection()
                            nextIntersect = intersections[21]
                        elif i == 0 or i == 2:
                            d = random_destination([17], candidate_destination, 0.8)
                            direction = origins[i].getDirection()
                            nextIntersect = intersections[9]
                        elif i == 5 or i == 7:
                            d = random_destination([9], candidate_destination, 0.8)
                            direction = origins[i].getDirection()
                            nextIntersect = intersections[17]
                        elif i == 8:
                            d = random_destination([15], candidate_destination, 0.8)
                            if constant.position_intersection[d][0] < -1500:
                                direction = np.array([-1, 0])
                                nextIntersect = intersections[23]
                            else:
                                direction = np.array([1, 0])
                                nextIntersect = intersections[2]
                        else:  # i==9:
                            d = random_destination([23], candidate_destination, 0.8)
                            if constant.position_intersection[d][0] <= 1500:
                                direction = np.array([-1, 0])
                                nextIntersect = intersections[1]
                            else:
                                direction = np.array([1, 0])
                                nextIntersect = intersections[15]

                        car: Car = Car(constant.MAX_VELOCITY, origins[i], constant.position_intersection[d],
                                       origins[i].getPosition(), direction, star_time, nextIntersect,
                                       intersections[d])

                        origins[i].cars.append(car)
                        star_time += interval
                        cars_append(car)

            links: List[Link] = []
            link_index = 0
            for i in range(len(constant.position_all_nodes)):
                for j in range(i + 1, len(constant.position_all_nodes)):
                    position_1 = constant.position_all_nodes[i]
                    position_2 = constant.position_all_nodes[j]
                    length = 0
                    difference = position_2 - position_1
                    if difference[0] * difference[1] != 0: continue  # 不在一条线上

                    if difference[0] != 0 and abs(difference[0]) <= constant.link_maxLength_horizontal:
                        length = abs(difference[0])
                    elif difference[1] != 0 and abs(difference[1]) <= constant.link_maxLength_vertical:
                        length = abs(difference[1])
                    if length == 0: continue
                    link = Link(length, position_1, position_2, link_index)
                    link_index += 1

                    for intersect in intersections:
                        if not (
                                (intersect.getPosition() == position_1).all()
                                or
                                (intersect.getPosition() == position_2).all()
                        ): continue

                        if (intersect.getPosition() == position_1).all():
                            difference = position_2 - position_1
                            link.node1_id = intersect.getIndex()
                        elif (intersect.getPosition() == position_2).all():
                            difference = position_1 - position_2
                            link.node2_id = intersect.getIndex()
                        else:
                            raise Exception("position")

                        if difference[0] == 0 and difference[1] > 0:
                            intersect.setLink_N(link)
                            link.setIntersection_S(intersect)
                        elif difference[0] == 0 and difference[1] < 0:
                            intersect.setLink_S(link)
                            link.setIntersection_N(intersect)
                        elif difference[0] > 0 and difference[1] == 0:
                            intersect.setLink_E(link)
                            link.setIntersection_W(intersect)
                        else:
                            intersect.setLink_W(link)
                            link.setIntersection_E(intersect)
                    links.append(link)
                    # print(link)
            del link_index

            for intersect in intersections:
                cur_hopMap = HopMap()
                if intersect.getLink_N().getIntersection_N():
                    cur_hopMap.intersections_oneHop.append(intersect.getLink_N().getIntersection_N())
                    cur_hopMap.links.append(intersect.getLink_N())
                    cur_hopMap.turns.append(np.array([0, 1]))
                if intersect.getLink_S().getIntersection_S():
                    cur_hopMap.intersections_oneHop.append(intersect.getLink_S().getIntersection_S())
                    cur_hopMap.links.append(intersect.getLink_S())
                    cur_hopMap.turns.append(np.array([0, -1]))
                if intersect.getLink_W().getIntersection_W():
                    cur_hopMap.intersections_oneHop.append(intersect.getLink_W().getIntersection_W())
                    cur_hopMap.links.append(intersect.getLink_W())
                    cur_hopMap.turns.append(np.array([-1, 0]))
                if intersect.getLink_E().getIntersection_E():
                    cur_hopMap.intersections_oneHop.append(intersect.getLink_E().getIntersection_E())
                    cur_hopMap.links.append(intersect.getLink_E())
                    cur_hopMap.turns.append(np.array([1, 0]))
                intersect.setHopMap(cur_hopMap)

            print("1.o-d route")
            for origin in intersections:
                for destination in intersections:
                    if (destination.getPosition() == origin.getPosition()).all(): continue

                    origin.destinations.append(destination)
                    splitRate = SplitRate(origin, destination)  # path
                    hopMap = origin.getHopMap()  # 下一跳
                    path_queue: List[Path] = []

                    for i in range(0, len(hopMap.intersections_oneHop)):
                        path_new = Path(origin, destination)
                        path_new.turn_count = 0
                        if hopMap.intersections_oneHop[i] in path_new.intersects: exit(1)
                        path_new.intersects.append(hopMap.intersections_oneHop[i])
                        path_new.links.append(hopMap.links[i])
                        path_new.turns.append(hopMap.turns[i])
                        path_new.distance = hopMap.links[i].getLength()
                        if (hopMap.intersections_oneHop[i].getPosition() == destination.getPosition()).all():
                            splitRate.paths.append(path_new)
                        else:
                            path_queue.append(path_new)

                    while len(path_queue) > 0:  # candidate中还有path
                        cur_path = path_queue.pop(0)
                        intersection = cur_path.intersects[len(cur_path.intersects) - 1]
                        cur_hopMap = intersection.getHopMap()
                        for i in range(0, len(cur_hopMap.intersections_oneHop)):
                            if cur_hopMap.intersections_oneHop[i] in cur_path.intersects: continue
                            path_new = Path(origin, destination)
                            if (cur_path.turns[-1] == cur_hopMap.turns[i]).all():
                                path_new.turn_count = cur_path.turn_count
                            else:
                                path_new.turn_count = cur_path.turn_count + 1
                                if path_new.turn_count > constant.turn_count_limit:
                                    continue

                            path_new.distance = cur_path.distance + cur_hopMap.links[i].getLength()

                            path_new.links = cur_path.links.copy()  # 这用浅copy，link对象不变
                            path_new.links.append(cur_hopMap.links[i])

                            path_new.turns = cur_path.turns.copy()
                            path_new.turns.append(cur_hopMap.turns[i])

                            path_new.intersects = cur_path.intersects.copy()
                            path_new.intersects.append(cur_hopMap.intersections_oneHop[i])

                            if (cur_hopMap.intersections_oneHop[i].getPosition()
                                == destination.getPosition()).all():
                                splitRate.paths.append(path_new)
                            else:
                                path_queue.append(path_new)

                    splitRate.paths.sort(key=lambda x: x.distance)

                    base_dis = splitRate.paths[0].distance
                    splitRate.base_distance = base_dis
                    splitRate.paths = list(
                        filter(lambda x: x.distance < constant.distance_limit + base_dis, splitRate.paths))

                    distances_ratio = np.array(
                        [path.distance / base_dis for path in splitRate.paths],
                        dtype=np.float)

                    splitting_rate = 1 / np.power(constant.split_rate_lmd * distances_ratio, constant.split_rate_beta)
                    splitting_rate = splitting_rate / np.sum(splitting_rate)
                    splitRate.setSplittingRate(splitting_rate)
                    origin.splitRates.append(splitRate)
                    origin.split_rate_map[destination.getIndex()] = splitRate

            print("1.o-d route done")
            self.cars = cars
            self.intersections = intersections
            self.origins = origins
            self.links = links

        else:
            self.cars = []
            self.intersections = []
            self.origins = []
            self.links = []

    def destroy(self):
        for car in self.cars:
            car.destroy()
        for original in self.origins:
            original.destroy()
        for intersect in self.intersections:
            intersect.destroy()
        for link in self.links:
            link.destroy()


def draw_link():
    links = []
    fig = plt.figure(figsize=(12, 9))
    with open("./chart/link_info.txt", mode="r")as f:
        for line in f.readlines():
            fields = line.strip().split(",")
            index = int(fields[0])
            pos1 = np.array(list(map(int, fields[1][1:-1].strip().split())))
            pos2 = np.array(list(map(int, fields[2][1:-1].strip().split())))
            node1 = int(fields[4])
            node2 = int(fields[5])

            ax = plt.subplot(111)
            plt.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]], color='k', linewidth=2)
            font = {'family': 'Microsoft YaHei',
                    'weight': 'bold',
                    'color': 'b',
                    'size': 12
                    }
            plt.text((pos1[0] + pos2[0]) / 2 * 1.01, (pos1[1] + pos2[1]) / 2 * 1.01, index,
                     horizontalalignment='center',
                     verticalalignment='center',
                     fontdict=font)

            font = {'family': 'Microsoft YaHei',
                    'weight': 'bold',
                    'color': 'red',
                    'size': 12
                    }

            cir = Circle(xy=pos1, radius=100, alpha=0.0, facecolor='none', edgecolor="k")
            ax.add_patch(cir)
            plt.text(pos1[0], pos1[1], node1,
                     horizontalalignment='center',
                     verticalalignment='center',
                     fontdict=font)

            cir = Circle(xy=pos2, radius=100, alpha=1.0, facecolor='none', edgecolor="k")
            ax.add_patch(cir)
            plt.text(pos2[0], pos2[1], node2,
                     horizontalalignment='center',
                     verticalalignment='center',
                     fontdict=font)
    plt.show()
