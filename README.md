# Proyecto Final: Navegación Autónoma con Planificación de Rutas en Webots

Robótica y Sistemas Autónomos 2026-01 — ICI 4150

**Integrantes:**
- Joaquín Fuenzalida
- Ignacio Ávila
- Sebástian Cruz
- Maximiliano Bustamante

## 1. Línea seleccionada

**Línea A: Planificación de rutas.** El robot calcula una ruta global desde una posición inicial hasta una meta usando **A\*** sobre una grilla de ocupación, y la ejecuta con seguimiento de waypoints más evitación reactiva de obstáculos.

## 2. Objetivo

Diseñar e implementar en Webots un robot diferencial (e-puck) capaz de planificar y ejecutar una ruta autónoma hacia una meta en dos escenarios de distinta complejidad, integrando control cinemático, percepción sensorial, odometría y planificación de rutas con A*.

## 3. Robot, sensores y actuadores

- **Robot:** e-puck (PROTO oficial de Webots), modelo diferencial de dos ruedas.
- **Actuadores:** `left wheel motor` / `right wheel motor`, controlados en velocidad (`setVelocity`), con conversión de `(v, ω)` a velocidades de rueda mediante el modelo cinemático diferencial (radio de rueda `R = 0.02 m`, distancia entre ruedas `L = 0.052 m`).
- **Sensores:**
  - 8 sensores de proximidad infrarrojos `ps0`–`ps7`, usados para detección frontal y lateral de obstáculos.
  - Encoders de rueda (`left wheel sensor`, `right wheel sensor`), usados para odometría.
- **Filtrado:** la lectura frontal cruda se suaviza con un filtro EMA (`α = 0.3`) y se fusiona con un filtro de Kalman 1D (predicción a partir del avance odométrico `delta_s`, corrección con la medición IR) para estimar la distancia frontal `d_hat` de forma más estable — mismo esquema usado en el Laboratorio 2.

## 4. Escenarios de prueba

### 4.1. Escenario simple (`escenario_simple.wbt`)
TODO: completar esta sección cuando el escenario quede cerrado. Debe incluir:
- Descripción del layout final (cantidad y disposición de obstáculos).
- Coordenadas de inicio y meta (`scenarios.py`).
- Una captura cenital del escenario en Webots.

> Nota interna: con la disposición actual (3 cajas con huecos de ~35 cm entre ellas) el A* probablemente atraviesa por el hueco más cercano en vez de rodear la fila. Si se busca que el robot ejecute una maniobra de "vuelta en U", las cajas deben quedar pegadas (sin hueco) para forzar el rodeo por un extremo.

### 4.2. Escenario complejo (`escenario_complejo.wbt`)
Laberinto simple de 5 "calles" verticales separadas por paredes (`WoodenBox`), cada una con una puerta de paso (offset distinto en cada pared), de modo que el robot debe avanzar y desviarse calle por calle hasta alcanzar la meta marcada con `YoubotFlag` en `(2.14, 1.66)`. El robot inicia en `(-2.2, 0.0)` y debe llegar únicamente hasta la meta (sin mapear ni explorar el resto del entorno).

TODO: agregar captura cenital del laberinto y, si se desea, el grafo/grilla de ocupación generado.

## 5. Algoritmo implementado

1. **Grilla de ocupación** ([occupancy_grid.py](controllers/controlador_navegacion_global/occupancy_grid.py)): discretiza el plano en celdas de `0.05 m`. Cada obstáculo rectangular del escenario se marca como ocupado, inflado por el radio del robot más un margen de seguridad (`ROBOT_RADIUS + SAFETY_MARGIN = 0.077 m`), de modo que cualquier celda libre es alcanzable sin colisión.
2. **A\*** ([planner.py](controllers/controlador_navegacion_global/planner.py)): búsqueda sobre la grilla con vecindad de 8 conexiones, heurística euclidiana, y un término adicional de penalización por baja "clearance" (distancia al obstáculo más cercano, vía BFS multi-fuente) para que la ruta prefiera el centro de los pasillos en vez de rozar las paredes. La ruta resultante se simplifica eliminando waypoints colineales.
3. **Seguimiento de ruta** ([path_follower.py](controllers/controlador_navegacion_global/path_follower.py)): control proporcional de rumbo (`K_HEADING = 1.0`) hacia el siguiente waypoint, con velocidad de crucero `0.08 m/s` escalada por `cos(error_rumbo)`. Avanza de waypoint en waypoint hasta entrar en tolerancia de meta (`0.20 m`).
4. **Evitación reactiva** ([obstacle_avoidance.py](controllers/controlador_navegacion_global/obstacle_avoidance.py)): máquina de estados (`IDLE → BACKOFF → TURN_LEFT/RIGHT → ESCAPE → IDLE`, con `REVERSE` si queda atrapado) que toma el control de los motores cuando la distancia frontal filtrada supera un umbral, independientemente de la ruta planificada — corrige desviaciones de odometría o detecta obstáculos no representados en el mapa.
5. **Odometría** ([kinematics.py](controllers/controlador_navegacion_global/kinematics.py)): integra los encoders de rueda paso a paso para estimar `(x, y, θ)`, usada tanto por el seguidor de ruta como por el filtro de Kalman de los sensores IR.

### Diagrama de flujo (controlador, por paso de simulación)

```
leer encoders → odometría (x, y, θ, Δs)
        ↓
leer sensores IR → filtro EMA + Kalman (d_hat)
        ↓
¿evitación de obstáculos activa? ──sí──→ aplicar (v_izq, v_der) de la FSM de evitación
        │no
        ↓
seguidor de ruta: calcular (v, ω) hacia el waypoint actual
        ↓
convertir (v, ω) → (v_izq, v_der) [cinemática diferencial]
        ↓
aplicar velocidades a los motores
        ↓
¿meta alcanzada? → reportar tiempo, pose estimada y distancia recorrida
```

## 6. Relación con los Laboratorios 1 y 2

- **Laboratorio 1:** el modelo cinemático diferencial (`v_robot`, `omega_robot`, `wheel_speeds` en [kinematics.py](controllers/controlador_navegacion_global/kinematics.py)) y la idea de descomponer un movimiento en `(v, ω)` y convertirlo a velocidades de rueda se reutilizan directamente para ejecutar la ruta planificada.
- **Laboratorio 2:** la lectura de sensores de distancia, el filtrado EMA + Kalman y la navegación reactiva de evitación de obstáculos se reutilizan casi sin cambios ([sensors.py](controllers/controlador_navegacion_global/sensors.py), [obstacle_avoidance.py](controllers/controlador_navegacion_global/obstacle_avoidance.py)), y se combinan ahora con la odometría para alimentar tanto el filtro de Kalman como el seguidor de ruta.
- **Extensión del proyecto final:** se añade la capa de navegación *global* (grilla de ocupación + A*) que faltaba en los laboratorios, donde la navegación era puramente reactiva/local.

## 7. Resultados y métricas

TODO — completar después de correr ambos escenarios (recomendado: ≥3 ejecuciones por escenario para sacar el % de éxito). El controlador ya imprime por consola, en cada ejecución:
- Largo de la ruta planificada por A* (`planner.path_length`).
- Tiempo de simulación hasta alcanzar la meta.
- Distancia recorrida real (acumulada por odometría).
- Pose estimada final.

Tabla sugerida (una fila por escenario, promediando varias corridas):

| Métrica | Escenario simple | Escenario complejo |
|---|---|---|
| Tiempo hasta la meta (s) | | |
| Longitud ruta planificada (m) | | |
| Longitud trayectoria ejecutada (m) | | |
| Diferencia ruta vs. trayectoria (m) | | |
| N° de activaciones de evitación de obstáculos | | |
| Colisiones | | |
| % de ejecuciones exitosas | | |

Falta también: registrar `(x, y)` estimado en cada paso (no se loguea actualmente, solo la pose final) si quieren graficar ruta planificada vs. trayectoria real — se puede agregar un logger CSV simple en el loop principal si lo necesitan.

## 8. Capturas, gráficos y video

TODO — ver checklist más abajo.

## 9. Instrucciones de ejecución

1. Abrir Webots (R2025a) y cargar `worlds/escenario_simple.wbt` o `worlds/escenario_complejo.wbt`.
2. El controlador `controlador_navegacion_global` ya está asignado al robot en cada mundo, con el nombre del escenario pasado como argumento (`controllerArgs`).
3. Ejecutar la simulación (▶). Por consola se imprime la ruta calculada por A* y, al llegar, el resumen de la ejecución.
4. Para correr el controlador apuntando a otro escenario manualmente, basta con cambiar el argumento en `controllerArgs` del nodo `E-puck` por `"escenario_simple"` o `"escenario_complejo"`.

## 10. Conclusiones, limitaciones y mejoras

TODO — completar al final, una vez evaluados ambos escenarios. Comentar al menos: estabilidad de la odometría (deriva acumulada), casos donde la evitación reactiva entra en conflicto con la ruta planificada, y posibles mejoras (replanificación dinámica, fusión con más sensores, etc.).
