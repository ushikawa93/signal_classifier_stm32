# Signal Classifier

Clasificador de formas de onda en tiempo real usando NanoEdge AI Studio sobre una placa NUCLEO-U575ZI-Q (STM32U5).

## Descripción

El DAC genera señales sinusoidal, cuadrada o triangular que el ADC mide y clasifica usando un modelo de ML embebido generado con NanoEdge AI Studio. El resultado se imprime por UART junto con un timestamp.

## Hardware

- NUCLEO-U575ZI-Q
- Cable entre pin de salida del DAC y pin de entrada del ADC

## Uso

- **Botón azul (B1):** cambia la señal generada por el DAC
- **UART (115200 8N1):** muestra la clasificación en tiempo real

## Modos

El firmware tiene tres modos seleccionables mediante `#define MODE` en `main.c`:
- `CAPTURE_MODE` — envía frames por UART en formato JSON para armar el dataset
- `CLASSIFY_MODE` — clasifica en tiempo real con el modelo de NanoEdge
- `CRONOMETER_MODE` - modo preeliminar para probar interrupciones y comunicacion UART
