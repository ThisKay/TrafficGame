import numpy as np


class Constant2:
    class ConstError(TypeError):
        pass

    class ConstCaseError(ConstError):
        pass

    def __setattr__(self, key, value):
        if key in self.__dict__:
            raise self.ConstError("Can't change const value!")
        # if not key.isupper():
        #     raise self.ConstCaseError('const "%s" is not all letters are capitalized' % key)
        self.__dict__[key] = value


constant = Constant2()
constant.ALPH = 0.8
constant.BETA = 6  # Wiki 中alpha=0.15， beta=4
constant.MAX_VELOCITY = 20  # free velocity 单位 m/s

constant.position_origin = np.array([[-3000, -2600], [3000, -2600],  # go_direction: N
                                     [-3200, -2400], [-3200, 2400],  # E
                                     [3200, -2400], [3200, 2400],  # W
                                     [-3000, 2600], [3000, 2600],  # S
                                     [-2300, 0], [2300, 0]]  # E and W
                                    , dtype=np.int32)

constant.position_intersection = np.array([[0, 0],
                                           [1500, 0], [-1500, 0],
                                           [0, 1200], [1500, 1200], [-1500, 1200],
                                           [0, -1200], [1500, -1200], [-1500, -1200],
                                           [-3000, -2400], [-1500, -2400], [0, -2400], [1500, -2400], [3000, -2400],
                                           [3000, -1200], [3000, 0], [3000, 1200], [3000, 2400],
                                           [1500, 2400], [0, 2400], [-1500, 2400], [-3000, 2400],
                                           [-3000, 1200], [-3000, 0], [-3000, -1200]]
                                          , dtype=np.int32)

constant.link_maxLength_horizontal = 1500
constant.link_maxLength_vertical = 1200

constant.position_all_nodes = np.array([[0, 0],
                                        [1500, 0], [-1500, 0],
                                        [0, 1200], [1500, 1200], [-1500, 1200],
                                        [0, -1200], [1500, -1200], [-1500, -1200],
                                        [-3000, -2400], [-1500, -2400], [0, -2400], [1500, -2400], [3000, -2400],
                                        [3000, -1200], [3000, 0], [3000, 1200], [3000, 2400],
                                        [1500, 2400], [0, 2400], [-1500, 2400], [-3000, 2400],
                                        [-3000, 1200], [-3000, 0], [-3000, -1200],

                                        [-3000, -2600], [-1500, -2600], [0, -2600], [1500, -2600], [3000, -2600],
                                        [3200, -2400], [3200, -1200], [3200, 0], [3200, 1200], [3200, 2400],
                                        [-3000, 2600], [-1500, 2600], [0, 2600], [1500, 2600], [3000, 2600],
                                        [-3200, -2400], [-3200, -1200], [-3200, 0], [-3200, 1200], [-3200, 2400]]
                                       , dtype=np.int32)

constant.target_link = [52, 50, 15, 16,
                        10, 11, 12, 46, 48,
                        13, 14, 41, 43,
                        7, 8, 9, 55, 57,
                        0, 1, 2, 3,
                        4, 5, 6, 37, 39,
                        22, 23, 24, 25,
                        17, 18, 19, 28, 30,
                        20, 21, 32, 34
                        ]

constant.region_link = [[52, 50, 15, 16],
                        [10, 11, 12, 46, 48],
                        [13, 14, 41, 43],
                        [7, 8, 9, 55, 57],
                        [0, 1, 2, 3],
                        [4, 5, 6, 37, 39],
                        [22, 23, 24, 25],
                        [17, 18, 19, 28, 30],
                        [20, 21, 32, 34]
                        ]

constant.split_rate_lmd = 1
# compare_control
constant.split_rate_beta = 3  # distance
constant.split_rate_gama = 1  # decay
constant.distance_limit = 3100
constant.control_patial = False
constant.agent_re_route = True
constant.re_route_inteval = 30
constant.agent_pa = 51
constant.turn_count_limit = 3
# log_control
constant.agent_print_road = False
constant.agent_print_car_num = True

constant.o_car_num = (10, 10,
                      10, 10,
                      10, 10,
                      10, 10,
                      110, 110)
constant.length_edg = 200

constant.delay_turn_straight = 1
constant.delay_turn_right = 10
constant.delay_turn_left = 15
constant.delay_turn_u = 20

constant.MAX_number_turn = 30

constant.interval_update = 1

constant.time_simulate = 5400

constant.Np = 30
constant.V_FREE = 72
constant.P_CRITICAL = 0.083
constant.P_MAX = 0.133
constant.am = 2.34
constant.namuta = 30
constant.tao = 10
constant.Ts = 10
constant.Tc = 60
constant.ki = 0.02
constant.Kp = 0.001
