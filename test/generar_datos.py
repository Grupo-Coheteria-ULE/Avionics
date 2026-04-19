#!/usr/bin/env python3
"""
Genera datos simulados de un vuelo parabolico.
"""

import csv
import math

ALT_EXPONENT = 0.1903
ALT_SCALE_M = 44330.0
P_BASE = 92000.0

def altitud_parabolica(t, t_apogeo, h_max):
    h = h_max * (1 - ((t - t_apogeo) / t_apogeo) ** 2)
    return max(0, h)

def presion_desde_altitud(h_m):
    if h_m <= 0:
        return P_BASE
    ratio = (1 - h_m / ALT_SCALE_M) ** (1 / ALT_EXPONENT)
    return P_BASE * ratio

T_VUELO = 60.0
T_APOGEO = 25.0
H_MAX = 1200.0
DT = 0.02

timestamps = []
presiones = []

t = 0.0
while t <= T_VUELO:
    h = altitud_parabolica(t, T_APOGEO, H_MAX)
    p = presion_desde_altitud(h)
    timestamps.append(int(t * 1e6))
    presiones.append(p)
    t += DT

OUTPUT_FILE = "data/datos_simulados.csv"

with open(OUTPUT_FILE, 'w', newline='', encoding='utf-8') as f:
    writer = csv.writer(f, delimiter=';')
    writer.writerow(['t', 'ax', 'ay', 'az', 'wx', 'wy', 'wz', 'T', 'p'])
    for i in range(len(timestamps)):
        writer.writerow([
            timestamps[i],
            0.0, 0.0, 1.0,
            0.0, 0.0, 0.0,
            20.0,
            presiones[i]
        ])

print(f"Datos generados: {len(timestamps)} muestras")
print(f"Archivo: {OUTPUT_FILE}")
print(f"Presion base (suelo): {presiones[0]:.2f} Pa")
print(f"Presion minima (apogeo): {min(presiones):.2f} Pa")
print(f" Tiempo apogeo: {T_APOGEO:.2f}s")
print(f" Altura maxima: {H_MAX:.0f}m")