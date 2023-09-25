import math
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
from IPython.display import display, Image as IPImage


speeds = {
    'sedan': 45,
    'van': 37.5,
    'bike': 55,
    'bus': 40
}

class Vehicle:
    def __init__(self, i_speed, a, pos, lane, tipo):
        self.speed = i_speed
        self.acceleration = a
        self.pos = pos
        self.lane = lane
        self.tipo = tipo


    # formula de aceleracion: v^2 = v0^2 + 2*a*d
    def frenar_acelerar(self, d, max_speed, semaforo):
        #print('  - speed: ', self.speed)
        #print('  - d: ', d)
        #print('  - pos: ', self.pos)
        #print('  - semaforo: ', semaforo.pos)
        #print('  - semaforo state: ', semaforo.state)
        
        #if self.pos in range(int(), semaforo.pos) and self.pos < semaforo.pos:    
        #print('  - semaforo: ',  self.pos, '(', semaforo.pos - (speeds[self.tipo] / 3.6 * 5)  ,semaforo.pos , ')' , semaforo.state)
        if self.pos >= semaforo.pos - (speeds[self.tipo] / 3.6) *5 and self.pos <= semaforo.pos:
            #print('  - flag 1: ', self.pos, self.speed)
            if semaforo.state == 'red':
                #print('  - frenar')
                self.speed = 0
            else:
                #print('  - acelerar')
                if self.speed == 0: # si paró en el semaforo, acelera de golpe
                    #self.speed = speeds[self.tipo] / 3.6 - 2*self.acceleration*5
                    temp = speeds[self.tipo] / 3.6
                    temp = temp**2
                    #print('  - temp: ', temp)
                    temp2 = 2*self.acceleration*5
                    #print('  - temp2: ', temp2)
                    self.speed = math.sqrt(temp - temp2)
                else:
                    if self.speed < max_speed:
                        if d > 5: # si la distancia es mayor a 10 metros, acelera
                            #print('  - flag 2: ', self.pos, self.speed)
                            temp = self.speed**2 + 2*self.acceleration*d
                            self.speed = math.sqrt(temp)
                        #self.speed = math.sqrt(self.speed**2 + 2*self.acceleration*d) 
                        elif d < 5 and d > 0:
                            #print('  - flag 3: ', self.pos, self.speed)
                            temp = self.speed**2 - 2*self.acceleration*d
                            if temp >= 0:
                                self.speed = math.sqrt(temp)
                            else:
                                self.speed = 0
                        elif d <= 0:
                            #print('  - flag 4: ', self.pos, self.speed)
                            # Frenar de golpe
                            self.speed = 0
                    else:
                        if d < 5 and d > 0:
                            #print('  - flag 5: ', self.pos, self.speed)
                            temp = self.speed**2 - 2*self.acceleration*d
                            if temp >= 0:
                                self.speed = math.sqrt(temp)
                            else:
                                self.speed = 0
                        elif d <= 0:
                            #print('  - flag 6: ', self.pos, self.speed)
                            # Frenar de golpe
                            self.speed = 0
        else:
            if self.speed < max_speed:
                if d > 5: # si la distancia es mayor a 10 metros, acelera
                    #print('  - flag 2: ', self.pos, self.speed)
                    temp = self.speed**2 + 2*self.acceleration*d
                    self.speed = math.sqrt(temp)
                #self.speed = math.sqrt(self.speed**2 + 2*self.acceleration*d) 
                elif d < 5 and d > 0:
                    #print('  - flag 3: ', self.pos, self.speed)
                    temp = self.speed**2 - 2*self.acceleration*d
                    if temp >= 0:
                        self.speed = math.sqrt(temp)
                    else:
                        self.speed = 0
                elif d <= 0:
                    #print('  - flag 4: ', self.pos, self.speed)
                    # Frenar de golpe
                    self.speed = 0
            else:
                if d < 5 and d > 0:
                    #print('  - flag 5: ', self.pos, self.speed)
                    temp = self.speed**2 - 2*self.acceleration*d
                    if temp >= 0:
                        self.speed = math.sqrt(temp)
                    else:
                        self.speed = 0
                elif d <= 0:
                    #print('  - flag 6: ', self.pos, self.speed)
                    # Frenar de golpe
                    self.speed = 0
        #print('-'*50)


class Road:
    def __init__(self, length, speed_limit, lanes=2):
        self.length = length
        self.speed_limit = speed_limit
        self.lanes = lanes


# Semaforo binario: sólo existe luz verde y roja, no amarilla
class Semaforo:
    def __init__(self, pos, state, red, green):
        self.pos = pos
        self.state = state
        self.red = red
        self.green = green
        self.timer = 0

    def semaforo_update(self, dt):
        if self.state == 'green':
            self.timer += dt
            if self.timer == self.green:
                self.state = 'red'
                self.timer = 0
        elif self.state == 'red':
            self.timer += dt
            if self.timer == self.red:
                self.state = 'green'
                self.timer = 0


def simulate_traffic(road, vehicles, semaforos, duration, dt):

    duration_s = duration * 3600 # convertir horas a segundos

    flow_speeds = []

    i = 0
    while i < duration_s and len(vehicles) > 0:
        #for v in vehicles:
        #    print(v.pos, '  -----  ', v.speed)
        #print('-'*50)


        i += dt
        for s in semaforos:
            s.semaforo_update(dt)
        
        for v in vehicles:
            
            distance_to_next = road.length
            next_semaforo = None
            # distancia al siguiente vehiculo o semaforo
            next_vehicle_distances = [abs(j.pos - v.pos) for j in vehicles if j != v and j.lane == v.lane]
            next_semaforo_distances = [abs(j.pos - v.pos) for j in semaforos]
            next_v_d = min(next_vehicle_distances) if next_vehicle_distances else road.length
            next_s_d = min(next_semaforo_distances) if next_semaforo_distances else road.length
            next_semaforo = semaforos[next_semaforo_distances.index(next_s_d)] 
            #quemascerca = "vehiculo" if next_v_d < next_s_d else "semaforo"
            #print('quemascerca: ', quemascerca)
            distance_to_next = min(next_v_d, next_s_d)
            v.frenar_acelerar(distance_to_next, road.speed_limit, next_semaforo)
            if (v.pos + v.speed * dt / 3.6) < road.length:
                v.pos += v.speed * dt / 3.6
            else:
                # Si el vehiculo llega al final de la carretera, se elimina del arreglo
                vehicles.remove(v)
        
        if len(vehicles) > 0:
            flow_speed = sum([v.speed for v in vehicles]) / len(vehicles)
            flow_speeds.append((i, flow_speed))        
    
    
        
    
    return flow_speeds




# conversión de km/h a m/s: 1 km/h * 1000 m/km * 1 h/3600 s = 1/3.6 m/s
# conversión de km/h^2 a m/s^2: 1 km/h^2 * 1000 m/km * 1 h/3600 s * 1 h/3600 s = 1000/(3600^2) m/s^2


# Simulacion

# Camino
road_l = 20 * 1000 # 100 km
speed_limit = 40 / 3.6 # km/h to m/s
road = Road(road_l, speed_limit, 2)

# Semaforo

semaforos = []
for s in range(0, road_l, 1000):
    semaforos.append(Semaforo(s, 'green', 30, 30))

# Vehiculos
    # Sedan - vi = 45 km/h, a = 3.5 m/s^2, space = 5 m
    # Van - vi = 37.5 km/h, a = 2.5 m/s^2, space = 7.5 m
    # Bike - vi = 55 km/h, a = 4 m/s^2, space = 3 m
    # Bus/Truck - vi = 40 km/h, a = 1.75 m/s^2, space = 10 m

sedan1 = Vehicle(45/3.6, 3.5, 0, -1, 'sedan')
sedan2 = Vehicle(45/3.6, 3.5, 10, -1, 'sedan') 
sedan3 = Vehicle(45/3.6, 3.5, 15, -1, 'sedan')
sedan4 = Vehicle(45/3.6, 3.5, 20, -1, 'sedan')
sedan5 = Vehicle(45/3.6, 3.5, 25, -1, 'sedan')
van1 = Vehicle(37.5/3.6, 2.5, 5, -1, 'van')
van2 = Vehicle(37.5/3.6, 2.5, 20, -1, 'van')
van3 = Vehicle(37.5/3.6, 2.5, 25, -1, 'van')
bike1 = Vehicle(55/3.6, 4, 20, -1, 'bike')
bike2 = Vehicle(55/3.6, 4, 15, -1, 'bike')
bike3 = Vehicle(55/3.6, 4, 10, -1, 'bike')
bike4 = Vehicle(55/3.6, 4, 5, -1, 'bike')
bus1 = Vehicle(40/3.6, 1.75, 40, -1, 'bus')
bus2 = Vehicle(40/3.6, 1.75, 30, -1, 'bus')

vehicles = [sedan1, sedan2, sedan3, van1, van2, bike1, bike2, bus1, bus2]

# assign lanes 
def assign_lane(vehicles, road, vehicle):
    position = vehicle.pos
    # Find the available lanes at the current position
    available_lanes = [lane for lane in range(road.lanes) if not any(v.pos == position and v.lane == lane for v in vehicles)]
    
    if not available_lanes:
        # If no lanes are available, move the vehicle ahead 5 meters until assigned
        vehicle.pos += 5
        assign_lane(vehicles, road, vehicle)
    else:
        # Assign the vehicle to the leftmost available lane
        vehicle.lane = available_lanes[0]

for v in vehicles:
    assign_lane(vehicles, road, v)



# Simulacion
duration = 2 # horas
dt = 5 # segundos

flow_speeds = simulate_traffic(road, vehicles, semaforos, duration, dt)

for f in flow_speeds:
    temp = f[1] * 3.6 # convertir m/s a km/h
    print(f[0], ' - ', temp)