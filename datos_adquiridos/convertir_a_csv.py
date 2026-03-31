"""
Convierte el log de PuTTY con frames JSON al formato CSV que espera NanoEdge AI Studio.
Uso: python convertir_a_csv.py datos_sinusoidal.txt sinusoidal.csv
"""

import re
import sys

def convertir(entrada, salida):
    with open(entrada, 'r') as f:
        contenido = f.read()

    # Busca todas las líneas con muestras
    patron = r'"Muestras":\s*([\d,\s]+?)\s*}'
    frames = re.findall(patron, contenido)

    if not frames:
        print("No se encontraron frames en el archivo.")
        return

    with open(salida, 'w') as f:
        for frame in frames:
            # Limpia y separa los valores
            valores = [v.strip() for v in frame.split(',') if v.strip()]
            f.write(','.join(valores) + '\n')

    print(f"✓ {len(frames)} frames exportados a '{salida}'")
    print(f"  Samples por frame: {len(valores)}")

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Uso: python convertir_a_csv.py <entrada.txt> <salida.csv>")
        sys.exit(1)
    convertir(sys.argv[1], sys.argv[2])
