import math

def steering_control(vehicle_x, vehicle_y, vehicle_yaw, pedestrian_x, pedestrian_y, pedestrian_yaw, dt, last_delta):
    """
    Diese Funktion berechnet den Lenkwinkel des Fahrzeugs basierend auf der
    relative Position zwischen Fahrzeug und Fußgänger.
    Es gibt hier zwei Regler für die Lenkungsregelung, Querabweichungsregelung
    und Gierwinkelsregelung.
    
    """
    # Parameter Definieren
    steering_range_max = math.pi / 6            # radiant, maximale Lenkungswinkel am Rad
    steering_ang_vel_max = 15                   # deg/s, maximale Lenksgeschwindigkeit
    K_lateral = 0.1                             # Querabweichungsregekung Proportionalverstärkung
    K_yaw = 0.8                                 # Gierwinkelsregelung Proportionalverstärkung
    
    # 1. Querabweichungsregelung
    direction_vehicle_to_pedestrian = [pedestrian_x - vehicle_x, pedestrian_y - vehicle_y]  # RichtungsVektor: Fahrzeug zu Fußgänger
    direction_vehicle = [math.cos(vehicle_yaw), math.sin(vehicle_yaw)]  # RichtungsVektor: Fahrzeug selbst

    # Kreuzprodukt, um zu bestimmen, ob der Fußgänger links oder rechts vom Fahrzeug liegt
    cross_product = direction_vehicle_to_pedestrian[1] * direction_vehicle[0] - direction_vehicle_to_pedestrian[0] * direction_vehicle[1]

    # Berechnung des Querdifferenz
    lateral_error = math.norm(direction_vehicle_to_pedestrian) * math.cos(pedestrian_yaw) * math.copysign(1, cross_product)

    # Berechnung der Steuergröße (Lenkwinkel)
    delta_lateral = K_lateral * lateral_error

    # 2. Gierwinkelsregelung
    yaw_to_target = math.atan2(pedestrian_y - vehicle_y, pedestrian_x - vehicle_x)  # Berechnung des Gierwinkels vom Fahrzeug zum Fußgänger

    # Berechnung des Gierwinkelsdifferenz
    yaw_error = yaw_to_target - vehicle_yaw

    # Berechnung der Steuergröße (Lenkwinkel)
    delta_yaw = K_yaw * yaw_error

    # neuer Lenkwinkel = Lenkwinkel der Querabweichungsregelung + Lenkwinkel der Gierwinkelsregelung
    delta = delta_lateral + delta_yaw
    
    # Begrenzung des Lenkwinkel und der Lenkgeschwindigkeit
    delta = max(min(delta, steering_range_max), -steering_range_max)
    
    delta_change = delta - last_delta
    max_delta_change = math.radians(steering_ang_vel_max) * dt  # Umwandlung von Grad/s zu Radiant/s und Multiplikation mit dt

    # Lenkgeschwindigkeit begrenzen
    if abs(delta_change) > max_delta_change:
        delta_change = math.copysign(max_delta_change, delta_change)
    
    # Lenkwinkel aktualisieren und im gültigen Bereich halten
    new_delta = last_delta + delta_change
    
    return new_delta


def velocity_control(last_velocity, v_max, acc_max, vehicle_x, vehicle_y, target_x, target_y, track_distance, dt):
    """
    Diese Funktion berechnet eine Zielgeschwindigkeit für das Fahrzeug,
    basierend auf den Positionsinformationen des Fahrzeugs und des vorausfahrenden Fußgängers.
    Die Zielgeschwindigkeit ermöglicht es dem Fahrzeug, dem Fußgänger bei gleichzeitiger Einhaltung eines sicheren Abstands zu folgen.
    """

    Kp_distance = 0.5  # P-regler für Abstandregelung
    Kp_vel = 2.0  # P-regler für Geschwindigkeitsregelung

    # Äußere Schleife: Abstandsregelung
    distance_to_target = math.sqrt((vehicle_x - target_x)**2 + (vehicle_y - target_y)**2)
    distance_diff = distance_to_target - track_distance

    # Wenn der Abstand zum Fußgänger kleiner als der Sicherheitsabstand,
    # soll_geschwindigkeit = 0, vollbremsen
    if distance_diff <= 0:
        velocity_ref = 0
    else:
        velocity_ref = Kp_distance * distance_diff  
        velocity_ref = min(velocity_ref, v_max)    # Beschränkung der Geschwindigkeit

    # Innere Schleife: Geschwindigkeitsregelung
    velocity_diff = velocity_ref - last_velocity
    acceleration_command = Kp_vel * velocity_diff  # P-regler für Geschwindigkeitsregelung

    # Beschränkung der Beschleunigung
    acceleration_command = min(acceleration_command, acc_max)
    
    # aktualisieren die neue Geschwindigkeit
    new_velocity = last_velocity + acceleration_command * dt

    # Beschränkung der neue Geschwindigkeit im Intervall [0, v_max]
    new_velocity = max(min(new_velocity, v_max), 0)

    return new_velocity, velocity_ref
