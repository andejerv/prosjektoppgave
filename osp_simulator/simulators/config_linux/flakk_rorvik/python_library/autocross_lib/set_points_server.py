from concurrent import futures
import logging

import numpy as np

import grpc
import set_points_pb2
import set_points_pb2_grpc

from velocity_profile import VelocityProfile

class SetPointsServicer(set_points_pb2_grpc.SetPointsServicer):
    def __init__(self):
        print('Initialize server')

        x_end   = 7_200
        t_total = 23 * 60

        t_accel = 2.0 * 60
        t_decel = 4.0 * 60

        self.velocity_profile = VelocityProfile(x_end, t_total, t_accel, t_decel)

        self.x_heading_adjust = 500

        self.U_min_heading_pid = 2.0 * 0.51444444444
        self.U_heading_reverse = 4 * 0.51444444444
        self.U_straight_line   = 6 * 0.5144444444 

    def GetSetPoints(self, request, context):
        t = request.t
        x = request.x
        y = request.y

        u = request.u
        v = request.v

        U = np.sqrt(u**2 + v**2)

        heading = request.heading

        loading = request.loading

        print('t, x, y, u, v, heading, loading:', np.round(t/60, 2), np.round(x, 1), np.round(y, 1), np.round(u, 4), np.round(v, 4), np.round(heading* 180 / np.pi, 2), np.round(loading, 1))

        if t > (self.velocity_profile.t_accel + self.velocity_profile.t_transit) and U < self.U_straight_line:
            self.velocity_profile.update_profile(t, x, 0.0)
        else:
            self.velocity_profile.update_profile(t, x, y)

        speed_set_point   = self.velocity_profile.get_target_velocity(t)

        if x > self.x_heading_adjust and U > self.U_min_heading_pid:
            drift_angle = np.arctan2(-v, u)
            heading_set_point = -np.arctan(y / x) + drift_angle
            heading_pid_on = True
        else:
            heading_set_point = 0.0
            heading_pid_on = True

        if t > (self.velocity_profile.t_accel + self.velocity_profile.t_transit) and U < self.U_heading_reverse:
            heading_gain_adjust = np.sign(loading)
        else:
            heading_gain_adjust = 1.0

        response_dict = {'speed_set_point': speed_set_point, 
                         'heading_set_point':heading_set_point,
                         'speed_pid_on':True,
                         'heading_pid_on':heading_pid_on,
                         'speed_gain_adjust': 1.0,
                         'heading_gain_adjust':heading_gain_adjust}
                         
        response = set_points_pb2.SetPointsResponse(**response_dict)

        return response
        

def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))

    set_points_pb2_grpc.add_SetPointsServicer_to_server(SetPointsServicer(), server)

    server.add_insecure_port('localhost:50051')
    server.start()
    server.wait_for_termination()


if __name__ == '__main__':
    logging.basicConfig()
    serve()