from traff_sim.constant import constant
from traff_sim.network import *
from traff_sim.visualization import *
import math
import cv2


class sim_Board:

    def __init__(self, **kwargs):
        self.width = 10
        self.height = 10
        # board states stored as a dict,
        # key: move as location on the board,
        # value: player as pieces type
        self.states = None
        self.n_in_row = int(kwargs.get('n_in_row', 5))
        self.players = [None, None]  # player1 and player2

    def init_board(self, start_player=0):

        self.current_player = sim_Agent(constant.agent_pa)
        # keep available moves in a list
        self.availables = list(range(3 * 3 + 1))
        self.states = np.zeros((4, self.width, self.height))
        self.last_move = None
        self.last_time = np.array([-1200] * 9)

    def move_to_location(self, move):
        """
        3*3 board's moves like:
        6 7 8
        3 4 5
        0 1 2
        and move 5's location is (1,2)
        """
        h = move // self.width
        w = move % self.width
        return [h, w]

    def location_to_move(self, location):
        if len(location) != 2:
            return -1
        h = location[0]
        w = location[1]
        move = h * self.width + w
        if move not in range(self.width * self.height):
            return -1
        return move

    def current_state(self):
        """return the board state from the perspective of the current player.
        state shape: 4*width*height
        """
        return self.states

    def do_move(self, move):
        current_time = self.current_player.clock
        if move < 9:
            self.last_time[move] = current_time
        guid_arr = np.argwhere(self.last_time > current_time - 900).reshape(-1)

        self.current_player.set_guid(guid_arr)
        self.current_player.tik_tok()
        current_time = self.current_player.clock
        new_state = self.current_player.current_state()
        new_state = cv2.resize(new_state.reshape((3, 3, 1)), (self.width, self.height)) \
            .reshape((1, self.width, self.height))
        new_state = np.concatenate((self.states[1:3, :, :], new_state), axis=0)  # t-2 t-1 t

        last_time_mat = self.last_time.reshape(3, 3)
        new_guid = np.zeros((3, 3))
        new_guid[last_time_mat > current_time - 900] = 1
        new_guid = new_guid.reshape((3, 3, 1))
        new_guid = cv2.resize(new_guid, (self.width, self.height), interpolation=cv2.INTER_NEAREST)
        new_guid = new_guid.reshape((1, self.width, self.height))
        new_state = np.concatenate((new_state, new_guid), axis=0)  # t-2 t-1 t guid

        self.states = new_state

    def has_a_winner(self):
        return self.current_player.has_winner()

    def game_end(self):
        """Check whether the game is ended or not"""
        if self.current_player.is_end():
            winner = self.has_a_winner()
            return True, winner
        return False, -1

    def get_clock(self):
        return self.current_player.clock


# 返回的都是3*3
class sim_Agent:
    def __init__(self, pa):
        self.pa = pa
        self.network = Network()
        self.clock = 1
        self.cars_run: List[Car] = []  # 正在跑的车
        self.cars_done: List[Car] = []  # 已经跑完的车

    # 执行5*60s
    def tik_tok(self):
        for car in self.cars_run:
            car.re_routed = False

        for time_sys in range(self.clock, self.clock + 300, constant.interval_update):  # 从1秒开始仿真到5400秒
            if constant.agent_re_route and time_sys % constant.re_route_inteval == 0:
                self.update_link_decay()
                self.apply_decay()

            if constant.agent_print_road and time_sys % 900 == 0:
                print(time_sys)
                writeToCsv(self.network.links, "links_shortest_{}_{}.csv".format(self.pa, time_sys // 900), "links")

            for i in range(0, len(self.network.origins)):
                origin = self.network.origins[i]
                while origin.leave_car_index < len(origin.cars) \
                        and origin.cars[origin.leave_car_index].getStartTime() <= time_sys:
                    car = origin.cars[origin.leave_car_index]
                    self.init_path(car)
                    self.cars_run.append(car)
                    origin.leave_car_index += 1
                del i, origin

            pop_car = []
            for car in self.cars_run:
                position_car = car.getPosition()
                position_destinate = car.getDestination()
                position_updated = position_car + car.getVelocity() * constant.interval_update * car.getDirection()
                difference_of_destinate = (position_destinate - position_car) * (position_destinate - position_updated)
                position_intersect = car.intersection_to_go[0].getPosition()
                difference_of_intersect = (position_intersect - position_car) * (position_intersect - position_updated)
                if not difference_of_destinate[0] + difference_of_destinate[1] > 0:
                    car.setEndTime(time_sys)
                    pop_car.append(car)
                    direction = car.getDirection()
                    if car.getLink():
                        if direction[0] + direction[1] > 0:
                            car.getLink().carOnRoad_positive.remove(car)
                        else:
                            car.getLink().carOnRoad_negative.remove(car)
                    del direction

                elif not difference_of_intersect[0] + difference_of_intersect[1] > 0:

                    directionToTurn = car.direction_to_go[0]  # 下一个方向
                    link_togo = car.link_to_go[0]  # 下一个路口
                    go = False
                    if directionToTurn[0] + directionToTurn[1] > 0:  # positive方向
                        if len(link_togo.carOnRoad_positive) < link_togo.getLength() * constant.P_MAX:
                            go = True
                    else:  # negative方向
                        if len(link_togo.carOnRoad_negative) < link_togo.getLength() * constant.P_MAX:
                            go = True

                    if not car.wait_done:
                        car.setWaitTurn(
                            get_time_turn(car.getDirection(), directionToTurn) - constant.interval_update
                        )
                        car.wait_done = True
                        if directionToTurn[0] + directionToTurn[1] > 0:
                            link_togo.carWait_positive.append(car)
                        else:
                            link_togo.carWait_negative.append(car)
                    elif car.getWaitTurn() > 0:
                        car.setWaitTurn(car.getWaitTurn() - constant.interval_update)
                    elif not go:
                        continue
                    else:
                        direction = car.getDirection()
                        if car.getLink():
                            if direction[0] + direction[1] > 0:
                                car.getLink().carOnRoad_positive.remove(car)
                            else:
                                car.getLink().carOnRoad_negative.remove(car)
                        del direction

                        link_togo = car.link_to_go[0]
                        directionToTurn = car.direction_to_go[0]
                        if directionToTurn[0] + directionToTurn[1] > 0:
                            carNum_onRoad = len(link_togo.carOnRoad_positive)
                            link_togo.carOnRoad_positive.append(car)
                            link_togo.carWait_positive.remove(car)
                            wait_car = [waitedCar for waitedCar in link_togo.carWait_positive
                                        if waitedCar.getWaitTurn() <= 0]
                            carNum_wait = len(wait_car) + 1
                        else:
                            carNum_onRoad = len(link_togo.carOnRoad_negative)
                            link_togo.carOnRoad_negative.append(car)
                            link_togo.carWait_negative.remove(car)
                            wait_car = [waitedCar for waitedCar in link_togo.carWait_negative
                                        if waitedCar.getWaitTurn() <= 0]
                            carNum_wait = len(wait_car) + 1

                        car.setDirection(directionToTurn)
                        car.setLink(link_togo)


                        car.setVelocity(
                            get_velocity_num(carNum_onRoad + carNum_wait,
                                             link_togo.getLength() * constant.P_CRITICAL,
                                             link_togo.getLength() * constant.P_MAX))
                        car.setPosition(position_intersect
                                        + car.getVelocity() * constant.interval_update * directionToTurn)
                        car.link_record.append(
                            [link_togo.getIndex(), 1 if directionToTurn[0] + directionToTurn[1] > 0 else 0])

                        car.turn_record.append(car.direction_to_go.pop(0))
                        car.link_record.append(car.link_to_go.pop(0))
                        car.intersection_to_go.pop(0)
                        car.wait_done = False

                        del directionToTurn, difference_of_destinate, difference_of_intersect
                    del position_intersect
                else:
                    car.setPosition(position_updated)
                del car, position_car, position_destinate, position_updated

            for car in pop_car:
                self.cars_done.append(car)
                self.cars_run.remove(car)
        self.clock += 300
        return self.cars_done, self.cars_run

    def update_link_decay(self):
        links = []
        link_carnum_arr = []
        for link in self.network.links:
            carNum_onRoad = len(link.carOnRoad_negative) + len(link.carOnRoad_positive)
            carNum_onWait = 0
            carNum_onWait += len([c for c in link.carWait_negative if c.getWaitTurn() <= 0])
            carNum_onWait += len([c for c in link.carWait_positive if c.getWaitTurn() <= 0])
            car_num = carNum_onRoad + carNum_onWait
            if link.getIndex() not in constant.target_link:
                assert carNum_onRoad == 0, "assert carNum_onRoad ==0"
                assert carNum_onWait == 0, "assert carNum_onWait ==0"
                continue
            links.append(link)
            link_carnum_arr.append(car_num)

        # print(link_carnum_arr)
        link_carnum_arr = np.array(link_carnum_arr)
        total_car_num = np.sum(link_carnum_arr)
        link_decay = []
        for i, curr_link_num in enumerate(link_carnum_arr):
            if constant.control_patial and links[i].getIndex() not in [0, 1]: continue
            if not links[i].in_guid: continue
            # print(links[i].getIndex())
            other_link_avg = (total_car_num - curr_link_num) / 39
            links[i].decay = (curr_link_num + 1) / (other_link_avg + 1)
            assert links[i].decay > 0, "assert links[i].decay >0," + links[i].decay
            # print([l.decay for l in links])

    def apply_decay(self):

        for intersect in self.network.intersections:
            for split_rate in intersect.splitRates:
                distances_ratio = np.array(
                    [math.pow(max([link.decay for link in path.links]), constant.split_rate_gama)
                     # * (path.distance + path.turn_count * 15) / split_rate.base_distance
                     * path.distance / split_rate.base_distance
                     for path in split_rate.paths])
                splitting_rate = 1 / np.power(constant.split_rate_lmd * distances_ratio, constant.split_rate_beta)
                splitting_rate = splitting_rate / np.sum(splitting_rate)
                split_rate.setSplittingRate(splitting_rate)

    def init_path(self, car, _intersect=None):
        first_intersect = _intersect
        if first_intersect == None:
            first_intersect = car.getFirstIntersection()
        end_intersect = car.getEndIntersection()
        split_rate = first_intersect.split_rate_map.get(end_intersect.getIndex(), None)
        if split_rate != None:
            path = random_path(split_rate.paths, split_rate.getSplittingRate())
            car.link_to_go = path.links.copy()
            car.direction_to_go = path.turns.copy()
            car.intersection_to_go = path.intersects.copy()
            assert len(car.intersection_to_go) == len(car.direction_to_go) + 1, "intersection len > direction len"
        else:
            assert _intersect == None, "_intersect == None"
            car.intersection_to_go = [car.getEndIntersection()]

    def current_state(self):
        state = np.zeros((3, 3), dtype=np.uint8)
        link_num = {}
        for link in self.network.links:
            carNum_onRoad = len(link.carOnRoad_negative) + len(link.carOnRoad_positive)
            carNum_onWait = 0
            carNum_onWait += len([c for c in link.carWait_negative if c.getWaitTurn() <= 0])
            carNum_onWait += len([c for c in link.carWait_positive if c.getWaitTurn() <= 0])
            car_num = carNum_onRoad + carNum_onWait
            if link.getIndex() not in constant.target_link:
                assert carNum_onRoad == 0, "assert carNum_onRoad ==0"
                assert carNum_onWait == 0, "assert carNum_onWait ==0"
                continue
            link_num[link.getIndex()] = car_num
        avg_rigion = {i: 0 for i in range(len(constant.region_link))}
        for region_idx in range(len(constant.region_link)):
            for link_idx in constant.region_link[region_idx]:
                avg_rigion[region_idx] += link_num[link_idx]
            avg_rigion[region_idx] /= len(constant.region_link[region_idx])
            state[region_idx // 3, region_idx % 3] = avg_rigion[region_idx]
        return state

    def set_guid(self, guid_arr):
        for link_idx in constant.target_link:
            self.network.links[link_idx].in_guid = False
        for i in range(len(guid_arr)):
            region_idx = guid_arr[i]
            for link_idx in constant.region_link[region_idx]:
                self.network.links[link_idx].in_guid = True

    def has_winner(self):
        all_time = 0
        car_num = len(self.cars_done)
        for car in self.cars_done:
            all_time += car.getEndTime() - car.getStartTime()
        avg_car = all_time / car_num

        std_car = 460
        ret = (std_car - avg_car) / std_car * 10
        ret = min(1, ret) if ret > 0 else max(-1, ret)
        return ret

    def is_end(self):
        return self.clock == 1 + 5400

