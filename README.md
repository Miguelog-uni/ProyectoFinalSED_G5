
# Sistema distribuido de monitorización de calidad del aire * Sistemas Empotrados Distribuidos - Grupo 5
Proyecto final de sistemas empotrados distribuidos, consistente en realizar un sistema de medición de calidad del aire por medio una placa ESP32-C3 y una ESP32-v4. 
Se utilizan dos nodos de ESP32 comunicados mediante MQTT, uno sensor y otro visualizador.

---

# Nodos

## 1. Nodo sensor

Proyecto:

```bash
nodo_sensor_pfinal
```

Funciones:

- Lee Si7021: temperatura y humedad.
- Lee SGP30: eCO₂ y TVOC.
- Detecta presencia con HC-SR04.
- Publica datos por MQTT solo cuando detecta presencia.

Tópicos publicados:

```text
sed/GXX/sensor/temp
sed/GXX/sensor/hum
sed/GXX/sensor/eco2
sed/GXX/sensor/tvoc
sed/GXX/status
```

---

## 2. Nodo visualización

Proyecto:

```bash
nodo_visualizacion
```

Funciones:

- Se suscribe a los tópicos MQTT del nodo sensor.
- Recibe temperatura, humedad, eCO₂ y TVOC.
- Activa LEDs según el estado de calidad del aire.

---

# Requisitos

ESP-IDF configurado:

```bash
source ~/esp/esp-idf/export.sh
```

Broker MQTT Mosquitto en el portátil:

```bash
sudo apt install mosquitto mosquitto-clients
```

En caso de necesitar configurar Mosquitto, se haría lo siguiente:

```bash
sudo nano /etc/mosquitto/conf.d/sed.conf
```

Se crea el archivo con el siguiente contenido:

```conf
listener 1883 0.0.0.0
allow_anonymous true
```

Se reinicia el servicio de MQTT:

```bash
sudo systemctl restart mosquitto
```

Se puede realizar la siguiente comprobación:

```bash
sudo ss -lntp | grep 1883
```

Debe aparecer:

```text
0.0.0.0:1883
```

---

# Obtener IP del portátil broker

```bash
hostname -I
```

Por ejemplo:

```text
10.213.149.184 ..... .....
```

En este caso, el broker MQTT será:

```text
mqtt://10.213.149.184:1883
```

---

# Configuración de cada nodo para usar MQTT:

En cada proyecto, abrir menu config e ir al menú "Example Configuration":

```bash
idf.py menuconfig
```

```text
Example Configuration
```

Realizar la siguiente configuración:

```text
WiFi SSID      -> nombre de la red WiFi/hotspot
WiFi Password  -> contraseña
Broker URL     -> mqtt://IP_DEL_PORTATIL:1883
```

Ejemplo:

```text
mqtt://10.213.149.184:1883
```
Hay que tener en cuenta que si la red no es de 2,4 GHz las placas no se podrán conectar al WiFi ni a MQTT.

---

# Lanzar nodo sensor

```bash
cd nodo_sensor_pfinal
idf.py set-target esp32 (depende de la placa utilizada)
idf.py build
idf.py -p PUERTO flash monitor
```

---

# Lanzar nodo visualización

En otra terminal:

```bash
cd nodo_visualizacion
idf.py set-target esp32c3 (depende de la placa utilizada)
idf.py build
idf.py -p /dev/ttyUSB1 flash monitor
```

---

# Probar manualmente el nodo visualización

En caso de que se quiera probar el nodo de visualización únicamente, se pueden enviar valores simulados:

```bash
mosquitto_pub -h 10.213.149.184 -p 1883 -t "sed/GXX/sensor/eco2" -m "500"

mosquitto_pub -h 10.213.149.184 -p 1883 -t "sed/GXX/sensor/eco2" -m "1000"

mosquitto_pub -h 10.213.149.184 -p 1883 -t "sed/GXX/sensor/eco2" -m "1800"
```

