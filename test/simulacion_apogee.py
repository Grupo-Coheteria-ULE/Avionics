#!/usr/bin/env python3
"""
Simulacion de deteccion de apogeo.

Lee datos del CSV y aplica los algoritmos de altitude y apogee
para verificar el funcionamiento.
"""

import csv
import math
import argparse
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ALT_EXPONENT = 0.1903
ALT_SCALE_M = 44330.0
APOGEE_MIN_ALT_MM = 100000
APOGEE_WINDOW = 10


def leer_csv(filepath: str):
    timestamps = []
    presiones = []
    with open(filepath, 'r', newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f, delimiter=';')
        for row in reader:
            t = int(row['t'])
            p = float(row['p'])
            timestamps.append(t / 1e6)
            presiones.append(p)
    return timestamps, presiones


def calcular_altitud(presiones: list[float]) -> list[float]:
    p_base = presiones[0]
    altitudes = []
    for p in presiones:
        ratio = p / p_base if p_base > 0 else 0
        if ratio > 0 and ratio <= 1:
            alt_m = ALT_SCALE_M * (1 - ratio) ** ALT_EXPONENT
            altitudes.append(alt_m * 1000)
        else:
            altitudes.append(0.0)
    return altitudes


def calcular_velocidad(altitudes: list[float], timestamps: list[float], ventana: int = 10) -> list[float]:
    velocidades = []
    n = len(altitudes)
    for i in range(n):
        if i < ventana:
            velocidades.append(0.0)
        else:
            sum_alt = 0.0
            sum_dt = 0.0
            for j in range(i - ventana, i):
                dt = timestamps[j] - timestamps[j-1] if j > 0 else 0.02
                if dt > 0:
                    sum_alt += altitudes[j] - altitudes[j-1]
                    sum_dt += dt
            if sum_dt > 0:
                vel = (sum_alt / sum_dt)
                velocidades.append(vel)
            else:
                velocidades.append(0.0)
    return velocidades


def detectar_apogee(altitudes: list[float], velocidades: list[float], timestamps: list[float]) -> tuple[str, int, float]:
    estado = "IDLE"
    pos_count = 0
    detect_idx = -1
    detect_time = 0.0

    for i in range(len(altitudes)):
        if estado == "IDLE":
            if altitudes[i] > APOGEE_MIN_ALT_MM:
                estado = "ASCENT"
                print(f"  [ASCENT] Altitud armada: {altitudes[i]/1000:.2f}m en t={timestamps[i]:.2f}s")

        elif estado == "ASCENT":
            if velocidades[i] < 0:
                pos_count += 1
                if pos_count >= APOGEE_WINDOW:
                    estado = "DETECTED"
                    detect_idx = i
                    detect_time = timestamps[i]
                    print(f"  [DETECTED] Apogeo detectado: t={detect_time:.2f}s")
                    print(f"  [DETECTED] Muestras bajando consecutivas: {pos_count}")
            else:
                if pos_count > 0:
                    print(f"  [RESET] Contador reiniciado en t={timestamps[i]:.2f}s (vel={velocidades[i]:.1f} mm/s)")
                pos_count = 0

    return estado, detect_idx, detect_time


def generar_graficos(timestamps, presiones, altitudes, velocidades, detect_time, output_dir):
    output_dir = Path(output_dir)

    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('Simulacion de vuelo - Datos del sensor', fontsize=14)

    t = timestamps

    axes[0].plot(t, [a/1000 for a in altitudes], 'b-', linewidth=0.5)
    axes[0].axhline(y=100, color='r', linestyle='--', linewidth=1, label='Umbral apogeo (100m)')
    if detect_time > 0:
        axes[0].axvline(x=detect_time, color='g', linestyle='-', linewidth=2, label=f'Apogeo t={detect_time:.2f}s')
    axes[0].set_xlabel('Tiempo (s)')
    axes[0].set_ylabel('Altitud (m)')
    axes[0].set_title('Altitud vs Tiempo')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, presiones, 'orange', linewidth=0.5)
    axes[1].axhline(y=presiones[0], color='b', linestyle='--', linewidth=1, label=f'Presion base: {presiones[0]:.0f} Pa')
    axes[1].set_xlabel('Tiempo (s)')
    axes[1].set_ylabel('Presion (Pa)')
    axes[1].set_title('Presion vs Tiempo')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, velocidades, 'r-', linewidth=0.5)
    axes[2].axhline(y=0, color='green', linestyle='--', linewidth=1, label='Velocidad cero')
    axes[2].axvline(x=detect_time, color='purple', linestyle='-', linewidth=2, label=f'Apogeo t={detect_time:.2f}s')
    axes[2].set_xlabel('Tiempo (s)')
    axes[2].set_ylabel('Velocidad (mm/s)')
    axes[2].set_title('Velocidad vertical vs Tiempo')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()

    output_file = output_dir / 'grafico_vuelo.png'
    plt.savefig(output_file, dpi=150)
    print(f"\nGrafica guardada en: {output_file}")

    plt.close()


def main():
    parser = argparse.ArgumentParser(description='Simulacion de deteccion de apogeo')
    parser.add_argument('--input', '-i', default='data/DATOS_1.CSV', help='Archivo CSV de entrada')
    parser.add_argument('--output', '-o', default='../outputs', help='Directorio de salida para graficas')
    args = parser.parse_args()

    input_path = Path(args.input)
    output_dir = Path(args.output)

    if not input_path.exists():
        print(f"Error: No se encuentra el archivo {input_path}")
        return 1

    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Leyendo datos de: {input_path}")
    timestamps, presiones = leer_csv(str(input_path))
    print(f"Datos leidos: {len(timestamps)} muestras")
    print(f"Presion base: {presiones[0]:.2f} Pa")
    print(f"Rango de presion: {min(presiones):.2f} - {max(presiones):.2f} Pa")

    print("\nCalculando altitud...")
    altitudes = calcular_altitud(presiones)
    print(f"Altitud maxima: {max(altitudes)/1000:.2f} m")

    print("\nCalculando velocidad vertical...")
    velocidades = calcular_velocidad(altitudes, timestamps)
    print(f"Velocidad maxima: {max(velocidades):.1f} mm/s")
    print(f"Velocidad minima: {min(velocidades):.1f} mm/s")

    print("\nEjecutando algoritmo de apogee...")
    estado, detect_idx, detect_time = detectar_apogee(altitudes, velocidades, timestamps)

    print(f"\n--- RESULTADO ---")
    print(f"Estado final: {estado}")
    if detect_time > 0:
        print(f"Tiempo de apogeo: {detect_time:.2f} s")
    else:
        print("Apogeo NO detectado")

    print(f"\nGenerando graficas en: {output_dir}")
    generar_graficos(timestamps, presiones, altitudes, velocidades, detect_time, output_dir)

    return 0


if __name__ == '__main__':
    exit(main())