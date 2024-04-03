import os
import json
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.signal import resample

def data_extraction_pedestrians():
    """
    Diese Funktion dient dazu, die benötigene X und Y Position des
    Fußgängers von der aufgenommene Daten zu auslesen. Außerdem führt auch die
    Vorverarbeitung der Daten durch, wie z.B erneute Abtastung, erneute
    Interpolation.
    """

    # Laden die Daten aus JSON-Datei
    current_file_path = os.path.dirname(__file__)
    json_file_path = os.path.join(current_file_path, '..', 'resource', 'AVL_Example.json')
    json_file_path = os.path.normpath(json_file_path)
    with open(json_file_path, 'r') as file:
        json_data = json.load(file)

    # Zur Speicherung der X-, Y- und Z-Koordinaten des Fußgängers
    data_x, data_y, data_z = [], [], []

    # Speicherung für jedes Zeitstempels in den JSON-Daten
    for timestamp in json_data.keys():
        keypoints = json_data[timestamp]

        # Suchen des Keypoints mit Keypoint = 0 und Extrahieren seiner Koordinaten
        for keypoint in keypoints:
            if keypoint['keypoint'] == 0:
                data_x.append(keypoint['x'])
                data_y.append(keypoint['y'])
                data_z.append(keypoint['z'])
                break

    # Umwandeln der Koordinatenwerte, X/Z/Y zu X/Y/Z, Umrechnung der Einheit
    X = np.array(data_x) / 1000.0  # [mm] -> [m]
    Y = np.array(data_z) / 1000.0  # [mm] -> [m]

    # Verschieben die ursprünliche Koordinaten zu (0,0)
    X = X + 1.0  # Beginnt bei x=0/y=0
    Y = Y - 8.0  # Beginnt bei x=0/y=0

    # Daten verarbeiten 1.Neuabtastung von Rohdaten mit gewünschter Abtastfrequenz, 2.Entfernung ungültiger Daten (Inf), 3.Erneute Interpolation -- Daten glätten
    original_freq = 60  # Hz, 1/60 s für jede Schritt, die originale Frequenz der Daten
    desired_freq = 10   # Hz, 1/10 s für jede Schritt(Simulation), die gewünschte Frequenz

    # Neuabtastung, Entfernung ungültiger Daten (Inf)
    resampled_x, resampled_y = resample_position(X, Y, original_freq, desired_freq)

    # erneuerte Interpolation, Die erneute Interpolation kann dazu beitragen, fehlende Zwischenpunkte zu ergänzen.
    smoothed_x, smoothed_y = path_smoothing(resampled_x, resampled_y)

    # Visualisierung
    plt.figure()
    plt.plot(X, Y, 'b.', markersize=5, label='Original Data')
    plt.plot(smoothed_x, smoothed_y, 'r', markersize=4, label='Smoothed Data')
    plt.title('Bewegungsdaten des Fußgängers')
    plt.xlabel('X Position in [m]')
    plt.ylabel('Y Position in [m]')
    plt.axis('equal')
    plt.legend()
    plt.show()

    return smoothed_x, smoothed_y

def resample_position(X, Y, original_freq, desired_freq):
    # Entfernung ungültiger Daten (Inf)
    valid_idx = ~(np.isinf(X) | np.isinf(Y))
    X = X[valid_idx]
    Y = Y[valid_idx]

    # Neuabtastung und Interpolation
    original_time_interval = 1 / original_freq
    desired_time_interval = 1 / desired_freq
    number_of_data = len(X)
    original_time_vector = np.arange(0, number_of_data * original_time_interval, original_time_interval)
    target_time = np.arange(0, original_time_vector[-1], desired_time_interval)

    f_x = interp1d(original_time_vector, X, kind='cubic')
    f_y = interp1d(original_time_vector, Y, kind='cubic')

    resampled_x = f_x(target_time)
    resampled_y = f_y(target_time)

    return resampled_x, resampled_y

def path_smoothing(resampled_x, resampled_y):
    # neue Interpolation, um die Daten zu gläten
    smoothed_x = np.convolve(resampled_x, np.ones(5)/5, mode='valid')
    smoothed_y = np.convolve(resampled_y, np.ones(5)/5, mode='valid')

    # ergängzen die verlorene Anfangs- und Endspunkte nach der Verarbeitung vom Movemean
    smoothed_x = np.concatenate(([resampled_x[0]], smoothed_x, [resampled_x[-1]]))
    smoothed_y = np.concatenate(([resampled_y[0]], smoothed_y, [resampled_y[-1]]))

    return smoothed_x, smoothed_y

if __name__ == '__main__':
    data_extraction_pedestrians()
