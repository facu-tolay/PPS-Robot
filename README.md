
# Valentin

## Introducción
Framework de un robot onmidireccional de cuatro ruedas que forma parte de las prácticas profesionales supervisadas y proyecto integrador, realizados dentro del Laboratorio de Arquitectura de Computadoras y pertenece a la carrera de Ingenería en Computación.


## Descripción

Este proyecto contiene severos modulos que respaldan al funcionamiento del robot, entre ellos se encuentran la cinemática del robot, el control de los motores, el controlador PID, la conexión a una red WiFi y la comunicación por MQTT.

<p align="center">
    <img src="Imagenes/giphy.gif" width="360" height="360">
</p>

## Ejecutar el proyecto

Clonar el repositorio

```bash
  git clone https://link-to-project
```

Entrar al directorio del proyecto

```bash
  cd valentin
```

Configurar el entorno para ESP-IDF

```bash
  . $HOME/esp/esp-idf/export.sh
```

Configura las variables del entorno

```bash
  idf.py menuconfig
```

Dentro de `Robot Settings` existen:

`WIFI_SSID`

`WIFI_PASS`

`BROKER_HOST`

`BROKER_PORT`

`ROBOT_ID`

Compilar el proyecto

```bash
  idf.py build
```

Grabar el binario

```bash
  idf.py -p /dev/ttyUSB0 flash
```
## Autores

- [@francovaira](https://www.github.com/francovaira)
- [@facu-tolay](https://www.github.com/facu-tolay)