import simpy
from random import seed, expovariate
from numpy.random import normal
from numpy.random import randint


class Glob:
    # General Data
    req_vehicles = 500000
    req_UAVs = 500000
    comm_range = 500
    Veh_Arrate = 0.2
    UAV_Arrate = 0.02777
    # Road
    Road_Distance = 10000
    Max_V_speed = 50
    Min_V_speed = 10
    Avg_V_speed = 38.23
    Std_V_speed = 11.469

    # Vehicles
    Tot_veh = 0
    Road_Veh = []
    V_enter = 0
    V_exits = 0
    # UAVs
    Max_U_speed = 100
    Min_U_speed = 50
    UAV_enter = 0
    UAV_exits = 0
    Road_UAV = []
    av_UAV = 0
    tot_uav = 0
    # Clusters
    cluster_id = 0
    Tot_Cluster = 0
    Road_Clusters = []
    # Packets
    Tot_packets = 0
    Tot_Av_packets = []
    t_Path = 0
    Tot_Paths = 0.0
    Tot_packet_delay = 0.0
    Path_available = False
    Direct_delivery = 0.0
    hop_delivery = 0.0

    @staticmethod
    def get_truncated_normal():
        speed = Glob.Std_V_speed * normal(0.0, 1.0) + Glob.Avg_V_speed
        while speed < 0 or speed > Glob.Max_V_speed or speed < Glob.Min_V_speed:
            speed = Glob.Std_V_speed * normal(0.0, 1.0) + Glob.Avg_V_speed
        return speed

    @staticmethod
    def check_total_path():
        if len(Glob.Road_UAV) != 0 and len(Glob.Road_Veh) >= 2:
            if Glob.av_UAV >= (len(Glob.Road_Clusters)-1):
                Glob.Path_available = True
            else:
                Glob.Path_available = False


class Cluster:

    def __init__(self, joining_vehicle):
        Glob.cluster_id += 1
        self.id = Glob.cluster_id
        self.cluster_vehicles = [joining_vehicle]
        Glob.Road_Clusters.append(self)
        Glob.Tot_Cluster += 1
        self.reference_veh = self.cluster_vehicles[0]

    def vehicle_join(self, joining_vehicle):
        self.cluster_vehicles.append(joining_vehicle)

    def vehicle_disjoin(self, leaving_vehicle):
        self.cluster_vehicles.remove(leaving_vehicle)


class Packet:
    __id = 0

    def __init__(self, vehicle_arrival_time):
        Packet.__id += 1
        self.id = Packet.__id
        self.arrival_time = vehicle_arrival_time
        self.packet_delay = 0.0
        Glob.Tot_packets += 1
        Glob.Tot_Av_packets.append(self)


class Vehicle:
    __id = 0

    def __init__(self, env):
        Vehicle.__id += 1
        self.id = Vehicle.__id
        self.speed = Glob.get_truncated_normal()
        self.t_arrival = env.now
        self.packet = Packet(self.t_arrival)
        self.av_packet = True
        self.departure_time = Glob.Road_Distance / self.speed
        self.abs_departure_time = self.departure_time + self.t_arrival
        self.time_to_connect = 0.0
        self.t_disconnect = 0.0
        self.is_connected = 0.0
        self.RSU_connect = 0
        self.traveled_time = 0.0
        self.traveled_distance = 0.0
        self.cluster = Cluster(self)
        Glob.Road_Veh.append(self)
        Glob.Tot_veh += 1
        Glob.check_total_path()
        self.run_process = env.process(self.traffic_run(env))

    def traffic_run(self, env):
        if len(Glob.Road_Veh) == 1:
            # self.abs_departure_time = self.departure_time + env.now
            yield env.timeout(self.departure_time)
            self.departure_event(env)
            Glob.Direct_delivery += 1
            return
        if Glob.Path_available:
            self.delivery_all(env)
        for v in reversed(Glob.Road_Veh):
            if v.id != self.id:
                self.check_connectivity(v, env)
                if Glob.Path_available:
                    self.delivery_all(env)
                if self.is_connected == 2:
                    # print("AT 2")
                    self.duration_to_connect(v, env)
                    yield env.timeout(self.time_to_connect)
                    self.connection_event(v, env)
                    self.duration_to_disconnect(v)
                    yield env.timeout(self.t_disconnect)
                    self.disconnection_event(v, env)
                    if env.now >= self.abs_departure_time:
                        self.departure_event(env)
                        return
                elif self.is_connected == 1:
                    # print("AT 1")
                    self.connection_event(v, env)
                    self.duration_to_disconnect(v)
                    yield env.timeout(self.t_disconnect)
                    self.disconnection_event(v, env)
                    if env.now >= self.abs_departure_time:
                        self.departure_event(env)
                        return
                elif self.is_connected == 0:
                    # print("AT 0")
                    if env.now >= self.abs_departure_time:
                        self.departure_event(env)
        if env.now < self.abs_departure_time:
            time_left = self.abs_departure_time - env.now
            yield env.timeout(time_left)
            self.departure_event(env)
            return

    def check_connectivity(self, v, env):
        if v.id != self.id:
            self.traveled_time = env.now - self.t_arrival
            self.traveled_distance = self.traveled_time * self.speed
            v.traveled_time = env.now - v.t_arrival
            v.traveled_distance = v.traveled_time * v.speed
            space_headway = v.traveled_distance - self.traveled_distance
            if v.abs_departure_time <= self.abs_departure_time or v.speed >= self.speed:
                if space_headway > Glob.comm_range:
                    self.is_connected = 0
                elif space_headway <= Glob.comm_range:
                    self.is_connected = 1
            elif v.abs_departure_time > self.abs_departure_time and space_headway > Glob.comm_range:
                self.is_connected = 2
            elif v.abs_departure_time > self.abs_departure_time and 0 < space_headway <= Glob.comm_range:
                self.is_connected = 1
            elif space_headway < 0:
                self.is_connected = 0

    def check_rsu_connectivity(self, env):
        self.traveled_time = env.now - self.t_arrival
        self.traveled_distance = self.traveled_time * self.speed
        if self.traveled_distance >= (Glob.Road_Distance - 2 * Glob.comm_range):
            self.RSU_connect = 1
        else:
            self.RSU_connect = 0

    def duration_to_connect(self, next_vehicle, env):
        self.traveled_time = env.now - self.t_arrival
        self.traveled_distance = self.traveled_time * self.speed
        next_vehicle.traveled_time = env.now - next_vehicle.t_arrival
        next_vehicle.traveled_distance = next_vehicle.traveled_time * next_vehicle.speed
        inter_distance = abs(next_vehicle.traveled_distance - self.traveled_distance)
        self.time_to_connect = (inter_distance - Glob.comm_range) / abs(self.speed - next_vehicle.speed)

    def duration_to_disconnect(self, connected_vehicle):
        self.t_disconnect = (2 * Glob.comm_range) / abs(self.speed - connected_vehicle.speed)

    def connection_event(self, last_vehicle, env):
        self.is_connected = 1
        packet_available = 0
        last_vehicle.cluster.vehicle_join(self)
        if len(self.cluster.cluster_vehicles) <= 1:
            Glob.Road_Clusters.remove(self.cluster)
        else:
            self.cluster.cluster_vehicles.clear()
            Glob.Road_Clusters.remove(self.cluster)

        self.cluster = last_vehicle.cluster
        Glob.check_total_path()
        if Glob.Path_available:
            self.delivery_all(env)
        last_vehicle.cluster.reference_veh.check_rsu_connectivity(env)
        if last_vehicle.cluster.reference_veh.RSU_connect == 1:
            for ve in last_vehicle.cluster.cluster_vehicles:
                if ve.av_packet:
                    packet_available += 1
                    ve.av_packet = False
                    Glob.hop_delivery += 1
                    Glob.t_Path += 1
            if packet_available != 0:
                self.packet.packet_delay = env.now - self.t_arrival
                Glob.Tot_packet_delay += self.packet.packet_delay

    def disconnection_event(self, last_vehicle, env):
        self.is_connected = 0
        for vehicle in last_vehicle.cluster.cluster_vehicles:
            if vehicle.id == self.id:
                last_vehicle.cluster.vehicle_disjoin(self)
        self.cluster = Cluster(self)
        self.cluster.cluster_vehicles.clear()
        self.cluster.vehicle_join(self)
        Glob.check_total_path()
        if Glob.Path_available:
            self.delivery_all(env)

    def departure_event(self, env):
        # print("departure event")
        Glob.V_exits += 1
        Glob.Road_Veh.remove(self)
        Glob.Road_Clusters.remove(self.cluster)
        Glob.check_total_path()
        if Glob.Path_available:
            self.delivery_all(env)
        if self.av_packet and self.is_connected == 0:
            Glob.Direct_delivery += 1
            self.RSU_connect = 1
            self.av_packet = False
            self.packet.packet_delay = self.abs_departure_time - self.t_arrival
            Glob.Tot_packet_delay += self.packet.packet_delay
        elif self.av_packet and self.is_connected == 1:
            packet_count = 0
            for veh in self.cluster.cluster_vehicles:
                if veh.av_packet:
                    veh.av_packet = False
                    packet_count += 1
                    Glob.hop_delivery += 1
                    Glob.t_Path += 1
            if packet_count != 0:
                self.packet.packet_delay = self.abs_departure_time - self.t_arrival
                Glob.Tot_packet_delay += self.packet.packet_delay

    def delivery_all(self, env):
        packet_ct = 0
        if len(Glob.Road_Veh) != 0 and len(Glob.Road_UAV) != 0:
            for v in Glob.Road_Veh:
                if v.av_packet:
                    packet_ct += 1
                    v.av_packet = False
            if packet_ct > 0:
                Glob.t_Path += packet_ct
                Glob.Tot_Paths += 1
                self.packet.packet_delay = env.now - Glob.Road_UAV[-1].arrival_time
                if self.packet.packet_delay > 0:
                    Glob.Tot_packet_delay += self.packet.packet_delay


class UAV:
    __id = 0

    def __init__(self, env):
        UAV.__id += 1
        self.id = UAV.__id
        self.speed = randint(Glob.Min_U_speed, Glob.Max_U_speed)
        self.arrival_time = env.now
        self.hover_time = Glob.Road_Distance / self.speed
        self.abs_hover_time = self.hover_time + self.arrival_time
        Glob.Road_UAV.append(self)
        Glob.av_UAV += 1
        Glob.tot_uav += 1
        self.run_process = env.process(self.uav_run(env))

    def uav_run(self, env):
        Glob.check_total_path()
        if Glob.Path_available and len(Glob.Road_Veh) >= 2:
            Glob.Road_Veh[-1].delivery_all(env)
        yield env.timeout(self.hover_time)
        Glob.av_UAV -= 1
        Glob.UAV_exits += 1
        Glob.Road_UAV.remove(self)
        Glob.check_total_path()
        if Glob.Path_available and len(Glob.Road_Veh) >= 2:
            Glob.Road_Veh[-1].delivery_all(env)
        return


def vehicle_arrivals(env, req_vehicles_exit):
    while Glob.V_enter <= int(req_vehicles_exit - 1):
        yield env.timeout(expovariate(Glob.Veh_Arrate))
        vehicle = Vehicle(env)
        Glob.V_enter += 1


def uav_arrivals(env, req_UAV_exit):
    while Glob.UAV_enter <= int(req_UAV_exit - 1):
        yield env.timeout(expovariate(Glob.UAV_Arrate))
        u = UAV(env)
        Glob.UAV_enter += 1
        if Glob.V_enter >= Glob.req_vehicles:
            break


if __name__ == '__main__':
    seed()
    enviro = simpy.Environment()
    enviro.process(vehicle_arrivals(enviro, Glob.req_vehicles))
    enviro.process(uav_arrivals(enviro, Glob.req_UAVs))
    enviro.run()
    print("FLOW RATE = {}\t".format(Glob.Veh_Arrate))
    print("UAV Flow Rate = {}\t".format(Glob.UAV_Arrate))
    print("Vehicles Entered = {}\t".format(Glob.V_enter))
    print("vehicles Exits = {}\t".format(Glob.V_exits))
    print("UAV enter = {}\t".format(Glob.UAV_enter))
    print("UAV exits = {}\t".format(Glob.UAV_exits))
    print("Average Delivery Delay in minutes = {}\t".format((Glob.Tot_packet_delay / Glob.req_vehicles) / 60))
    print("Total vehicles = {}\t".format(Glob.Tot_veh))
    print("Path availability = {}\t".format(Glob.t_Path/Glob.Tot_veh))
