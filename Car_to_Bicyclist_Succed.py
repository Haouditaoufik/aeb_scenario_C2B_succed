import carla
import time
import math
import pygame
import sys
import os
import socket
import struct
import matplotlib.pyplot as plt
from collections import deque
import threading

def get_speed(vehicle):
    velocity = vehicle.get_velocity()
    return math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

def calculate_ttc(distance, relative_velocity):
    """Calcule le Time-To-Collision"""
    if relative_velocity <= 0:
        return float('inf')  # Pas de collision si vitesse relative <= 0
    return distance / relative_velocity

def setup_tcp_server(port=9001):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('localhost', port))
    sock.listen(1)
    print(f"En attente de connexion Simulink sur le port {port}...")
    try:
        connection, addr = sock.accept()
        connection.settimeout(0.1)  # Timeout plus court comme dans le script 2
        print(f"Connexion Simulink établie depuis {addr}")
        return connection, sock
    except Exception as e:
        print(f"Erreur connexion TCP: {e}")
        sock.close()
        return None, None

def send_data(conn, data):
    try:
        values = [data['MIO_Distance'], data['MIO_Velocity'], data['Ego_Velocity']]
        for val in values:
            conn.sendall(struct.pack('d', float(val)))
        print(f"Données envoyées: {values}")
    except Exception as e:
        print(f"Erreur d'envoi: {e}")
        raise

def receive_data(conn):
    try:
        values = []
        for _ in range(4):  # Recevoir 4 valeurs comme dans le script 2
            try:
                data = conn.recv(8)
                if len(data) < 8:
                    print("Données incomplètes reçues, réessai...")
                    time.sleep(0.01)
                    continue
                values.append(struct.unpack('d', data)[0])
            except socket.timeout:
                print("Timeout en attente de données")
                return {
                    'egoCarStop': False,
                    'FCW_Activate': False,
                    'Deceleration': 0.0,
                    'AEB_Status': False
                }
        
        # Structure de données simplifiée
        result = {
            'egoCarStop': bool(round(values[0])),
            'FCW_Activate': bool(round(values[1])),
            'Deceleration': float(values[2]),
            'AEB_Status': bool(round(values[3]))
        }
        print(f"Données reçues: {result}")
        return result
        
    except Exception as e:
        print(f"Erreur de réception: {e}")
        return {
            'egoCarStop': False,
            'FCW_Activate': False,
            'Deceleration': 0.0,
            'AEB_Status': False
        }

def initialize_carla():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.load_world('Town03')
    time.sleep(1)
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)
    return client, world

def setup_weather_rain(world):
    weather = carla.WeatherParameters(
        cloudiness=80.0,
        precipitation=60.0,
        precipitation_deposits=90.0,
        wind_intensity=40.0,
        sun_azimuth_angle=70.0,
        sun_altitude_angle=70.0,
        fog_density=20.0,
        fog_distance=50.0,
        wetness=80.0
    )
    world.set_weather(weather)

def spawn_actors(world):
    bp_lib = world.get_blueprint_library()

    ego_bp = bp_lib.find('vehicle.audi.tt')
    ego_spawn = carla.Transform(carla.Location(x=8.0, y=-80.0, z=0.3), carla.Rotation(yaw=90))

    # Cycliste : plus reculé
    cyclist_bp = bp_lib.find('vehicle.bh.crossbike')
    cyclist_spawn = carla.Transform(carla.Location(x=8.0, y=-40.0, z=0.3), carla.Rotation(yaw=-86))

    # Nettoyage
    for actor in world.get_actors().filter('vehicle.*'):
        actor.destroy()
    time.sleep(0.5)

    ego = world.spawn_actor(ego_bp, ego_spawn)
    cyclist = world.spawn_actor(cyclist_bp, cyclist_spawn)

    return ego, cyclist

def control_cyclist(cyclist, sim_time):
    if sim_time > 2.0:
        cyclist.apply_control(carla.VehicleControl(throttle=0.4, steer=0.0))
    else:
        cyclist.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))

def control_ego(ego, sim_time, sim_data, collision):
    control = carla.VehicleControl()
    
    if sim_data['AEB_Status'] or sim_data['egoCarStop'] or collision:
        control.throttle = 0.0
        control.brake = 1.0
    else:
        if sim_time > 2.5:
            control.throttle = max(0.3, 0.5 - sim_data['Deceleration'])
            control.brake = sim_data['Deceleration']
            control.steer = 0.0
        else:
            control.throttle = 0.0
            control.brake = 1.0
    
    return control

class RealTimePlotter:
    def __init__(self, max_points=200):
        self.max_points = max_points
        self.times = deque(maxlen=max_points)
        self.distances = deque(maxlen=max_points)
        self.ttc_values = deque(maxlen=max_points)
        self.ego_speeds = deque(maxlen=max_points)
        self.cyclist_speeds = deque(maxlen=max_points)
        
        # Configuration matplotlib pour éviter les conflits de thread
        plt.switch_backend('TkAgg')  # Backend plus stable
        plt.ion()
        
        try:
            self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(10, 8))
            self.fig.suptitle('Analyse AEB en Temps Réel - Conditions Pluvieuses')
            
            # Configuration des axes
            self.ax1.set_ylabel('Distance (m)')
            self.ax1.set_title('Distance Ego-Cycliste')
            self.ax1.grid(True)
            
            self.ax2.set_ylabel('TTC (s)')
            self.ax2.set_title('Time-To-Collision')
            self.ax2.grid(True)
            self.ax2.set_ylim(0, 10)
            
            self.ax3.set_ylabel('Vitesse (m/s)')
            self.ax3.set_xlabel('Temps (s)')
            self.ax3.set_title('Vitesses des Véhicules')
            self.ax3.grid(True)
            
            plt.tight_layout()
            plt.show(block=False)
            self.plotting_enabled = True
        except Exception as e:
            print(f"[WARNING] Impossible d'initialiser matplotlib: {e}")
            print("[INFO] Continuer sans graphiques temps réel")
            self.plotting_enabled = False
    
    def update(self, time_val, distance, ttc, ego_speed, cyclist_speed):
        if not self.plotting_enabled:
            return
            
        try:
            self.times.append(time_val)
            self.distances.append(distance)
            self.ttc_values.append(min(ttc, 10))
            self.ego_speeds.append(ego_speed)
            self.cyclist_speeds.append(cyclist_speed)
            
            # Mise à jour moins fréquente pour éviter les conflits
            if len(self.times) % 5 == 0:  # Mise à jour tous les 5 points
                self._update_plots()
        except Exception as e:
            print(f"[WARNING] Erreur matplotlib: {e}")
            self.plotting_enabled = False
    
    def _update_plots(self):
        try:
            # Mise à jour des graphiques
            self.ax1.clear()
            self.ax1.plot(list(self.times), list(self.distances), 'b-', linewidth=2, label='Distance')
            self.ax1.axhline(y=2.5, color='r', linestyle='--', label='Seuil collision')
            self.ax1.set_ylabel('Distance (m)')
            self.ax1.set_title('Distance Ego-Cycliste')
            self.ax1.legend()
            self.ax1.grid(True)
            
            self.ax2.clear()
            self.ax2.plot(list(self.times), list(self.ttc_values), 'r-', linewidth=2, label='TTC')
            self.ax2.axhline(y=1.5, color='orange', linestyle='--', label='TTC critique')
            self.ax2.set_ylabel('TTC (s)')
            self.ax2.set_title('Time-To-Collision')
            self.ax2.set_ylim(0, 10)
            self.ax2.legend()
            self.ax2.grid(True)
            
            self.ax3.clear()
            self.ax3.plot(list(self.times), list(self.ego_speeds), 'g-', linewidth=2, label='Ego')
            self.ax3.plot(list(self.times), list(self.cyclist_speeds), 'm-', linewidth=2, label='Cycliste')
            self.ax3.set_ylabel('Vitesse (m/s)')
            self.ax3.set_xlabel('Temps (s)')
            self.ax3.set_title('Vitesses des Véhicules')
            self.ax3.legend()
            self.ax3.grid(True)
            
            plt.draw()
            plt.pause(0.001)  # Pause très courte
        except Exception as e:
            print(f"[WARNING] Erreur mise à jour graphique: {e}")
            self.plotting_enabled = False

def main():
    conn = None
    sock = None
    world = None
    
    try:
        pygame.init()
        screen = pygame.display.set_mode((500, 300))
        pygame.display.set_caption("HUD - Simulation AEB Pluie")
        font = pygame.font.Font(None, 24)
        font_small = pygame.font.Font(None, 20)

        client, world = initialize_carla()
        
        # Configuration météo pluie
        setup_weather_rain(world)
        
        ego, cyclist = spawn_actors(world)
        
        # Connexion TCP simplifiée
        conn, sock = setup_tcp_server(9001)
        
        # Initialisation du traceur temps réel
        plotter = RealTimePlotter()

        clock = pygame.time.Clock()
        sim_time = 0.0
        collision = False
        
        # Structure de données par défaut simplifiée
        sim_data = {
            'egoCarStop': False,
            'FCW_Activate': False,
            'Deceleration': 0.0,
            'AEB_Status': False
        }

        print("[INFO] Simulation démarrée avec conditions pluvieuses")

        while True:
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    return

            world.tick()
            sim_time += 0.05

            control_cyclist(cyclist, sim_time)

            distance = ego.get_location().distance(cyclist.get_location())
            ego_speed = get_speed(ego)
            cyclist_speed = get_speed(cyclist)
            
            # Calcul vitesse relative et TTC
            relative_velocity = ego_speed - cyclist_speed
            ttc = calculate_ttc(distance, relative_velocity)

            # Communication TCP simplifiée
            if conn:
                try:
                    send_data(conn, {
                        'MIO_Distance': distance,
                        'MIO_Velocity': cyclist_speed,
                        'Ego_Velocity': ego_speed
                    })
                    sim_data = receive_data(conn)
                except Exception as e:
                    print(f"[TCP ERROR]: {e}")
                    # Continuer avec les dernières valeurs connues

            # Contrôle ego simplifié
            ctrl = control_ego(ego, sim_time, sim_data, collision)
            ego.apply_control(ctrl)

            if distance < 2.5 and not collision:
                print("[COLLISION] DETECTED!")
                collision = True

            # Mise à jour du graphique temps réel
            plotter.update(sim_time, distance, ttc, ego_speed, cyclist_speed)

            # HUD simplifié
            screen.fill((20, 20, 40))
            
            title_text = "SIMULATION AEB - CONDITIONS PLUVIEUSES"
            title = font.render(title_text, True, (255, 255, 0))
            screen.blit(title, (10, 10))
            
            hud_lines = [
                f"Vitesse Ego: {ego_speed*3.6:.1f} km/h",
                f"Vitesse Cycliste: {cyclist_speed*3.6:.1f} km/h",
                f"Distance: {distance:.2f} m",
                f"TTC: {ttc:.2f} s" if ttc != float('inf') else "TTC: ∞",
                "",
                f"Accélérateur: {ctrl.throttle:.2f}",
                f"Frein: {ctrl.brake:.2f}",
                f"AEB Actif: {sim_data['AEB_Status']}",
                f"FCW: {sim_data['FCW_Activate']}",
                f"Collision: {collision}",
                f"TCP: {'Actif' if conn else 'Inactif'}"
            ]
            
            for i, text in enumerate(hud_lines):
                if text == "":
                    continue
                color = (255, 255, 255)
                if "Collision: True" in text:
                    color = (255, 0, 0)
                elif "AEB Actif: True" in text or "FCW: True" in text:
                    color = (255, 165, 0)
                elif "TTC:" in text and ttc < 2.0 and ttc != float('inf'):
                    color = (255, 0, 0)
                
                txt = font_small.render(text, True, color)
                screen.blit(txt, (10, 45 + i * 22))
            
            pygame.display.flip()

            # Caméra spectateur
            spectator = world.get_spectator()
            ego_location = ego.get_location()
            ego_rotation = ego.get_transform().rotation
            
            yaw_rad = math.radians(ego_rotation.yaw)
            camera_x = ego_location.x - 17 * math.cos(yaw_rad)
            camera_y = ego_location.y - 17 * math.sin(yaw_rad)
            camera_z = ego_location.z + 12
            camera_location = carla.Location(x=camera_x, y=camera_y, z=camera_z)
            camera_rotation = carla.Rotation(pitch=-25, yaw=ego_rotation.yaw, roll=0)
            
            spectator.set_transform(carla.Transform(camera_location, camera_rotation))

            clock.tick(20)

    except KeyboardInterrupt:
        print("\n[INFO] Arrêt demandé par l'utilisateur")
    except Exception as e:
        print(f"Erreur principale : {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Nettoyage...")
        try:
            if conn:
                conn.close()
            if sock:
                sock.close()
            if world:
                for actor in world.get_actors().filter('vehicle.*'):
                    actor.destroy()
        except Exception as e:
            print(f"Erreur nettoyage: {e}")
        
        try:
            pygame.quit()
        except:
            pass
        
        try:
            plt.close('all')
        except:
            pass

if __name__ == "__main__":
    print(">>> Lancement du test AEB vs Cycliste ")
    print(">>> Fonctionnalités: Pluie, Communication TCP , Graphiques temps réel")
    main()