# Proyecto Final: Navegación Autónoma con Planificación de Rutas en Webots

Robótica y Sistemas Autónomos 2026-01, ICI 4150

Integrantes:
- Joaquín Fuenzalida
- Ignacio Ávila
- Sebástian Cruz
- Maximiliano Bustamante

## 1. Línea seleccionada

Elegimos la Línea A: planificación de rutas. El robot calcula una ruta global desde su posición inicial hasta una meta usando A* sobre una grilla de ocupación, y la ejecuta siguiendo waypoints mientras evita obstáculos de forma reactiva.

## 2. Objetivo

Implementar en Webots un robot diferencial (e-puck) que planifique y ejecute una ruta autónoma hacia una meta en dos escenarios de distinta complejidad, integrando control cinemático, percepción sensorial, odometría y planificación con A*.

## 3. Robot, sensores y actuadores

Usamos el e-puck, el PROTO oficial de Webots para robots diferenciales de dos ruedas. Los motores (`left wheel motor` y `right wheel motor`) se controlan en velocidad, convirtiendo `(v, ω)` a velocidades de rueda con el modelo cinemático diferencial (radio de rueda `R = 0.02 m`, distancia entre ruedas `L = 0.052 m`).

Para percibir el entorno usamos los 8 sensores de proximidad infrarrojos del e-puck (`ps0` a `ps7`) y los encoders de las ruedas (`left wheel sensor`, `right wheel sensor`) para la odometría. La lectura frontal cruda se suaviza con un filtro EMA (`α = 0.3`) y se combina con un filtro de Kalman 1D, que predice a partir del avance odométrico y corrige con la medición IR, para estimar la distancia frontal `d_hat` de forma más estable. Es el mismo esquema que usamos en el Laboratorio 2.

## 4. Escenarios de prueba

### 4.1. Escenario simple (`escenario_simple.wbt`)

Un solo obstáculo: tres cajas de madera pegadas entre sí, sin huecos, que forman una pared continua de 1.8 m en `y ≈ 0.05`. El robot parte en `(0.11647, 0.881111)`, arriba de la pared, y la meta está en `(0.0, -1.5)`, abajo. Como la pared bloquea el paso directo, el A* tiene que rodear uno de sus extremos, lo que genera la maniobra de "vuelta en U" que planteamos para este escenario.

![Escenario simple](Image/Imagen%20simple.png)

### 4.2. Escenario complejo (`escenario_complejo.wbt`)

Un laberinto simple de 5 "calles" verticales separadas por paredes (`WoodenBox`), cada una con una puerta de paso en una posición distinta, de modo que el robot tiene que avanzar y desviarse calle por calle hasta llegar a la meta, marcada con un `YoubotFlag` en `(2.14, 1.66)`. El robot parte en `(-2.2, 0.0)` y solo tiene que llegar a la meta, sin mapear ni explorar el resto del entorno.

![Escenario complejo](Image/Imagen%20Complejo.png)

TODO: si queremos, podemos agregar también el grafo o la grilla de ocupación generada.

## 5. Algoritmo implementado

La navegación se arma en capas:

1. Grilla de ocupación ([occupancy_grid.py](controllers/controlador_navegacion_global/occupancy_grid.py)). Discretiza el plano en celdas de 0.05 m y marca como ocupado cada obstáculo del escenario, inflado por el radio del robot más un margen de seguridad (`ROBOT_RADIUS + SAFETY_MARGIN = 0.077 m`), así cualquier celda libre es alcanzable sin chocar.
2. A* ([planner.py](controllers/controlador_navegacion_global/planner.py)). Busca sobre la grilla con vecindad de 8 conexiones y heurística euclidiana. Le agregamos una penalización por baja "clearance" (la distancia al obstáculo más cercano, calculada con un BFS multi-fuente) para que la ruta prefiera el centro de los pasillos en vez de rozar las paredes. Al final simplificamos la ruta eliminando waypoints colineales.
3. Seguimiento de ruta ([path_follower.py](controllers/controlador_navegacion_global/path_follower.py)). Control proporcional de rumbo (`K_HEADING = 1.0`) hacia el siguiente waypoint, con velocidad de crucero de 0.08 m/s escalada por el coseno del error de rumbo. Avanza waypoint por waypoint hasta entrar en la tolerancia de meta (0.20 m).
4. Evitación reactiva ([obstacle_avoidance.py](controllers/controlador_navegacion_global/obstacle_avoidance.py)). Una máquina de estados (`IDLE → BACKOFF → TURN_LEFT/RIGHT → ESCAPE → IDLE`, con `REVERSE` si el robot queda atrapado) que toma el control de los motores cuando la distancia frontal filtrada supera un umbral, sin importar la ruta planificada. Corrige desviaciones de odometría y detecta obstáculos que no están en el mapa.
5. Odometría ([kinematics.py](controllers/controlador_navegacion_global/kinematics.py)). Integra los encoders paso a paso para estimar `(x, y, θ)`, y ese dato lo usan tanto el seguidor de ruta como el filtro de Kalman de los sensores.

### Diagrama de flujo (controlador, por paso de simulación)

![Diagrama de flujo](Image/Diagrama%20de%20flujo.png)

## 6. Relación con los Laboratorios 1 y 2

Del Laboratorio 1 reutilizamos directamente el modelo cinemático diferencial (`v_robot`, `omega_robot`, `wheel_speeds` en [kinematics.py](controllers/controlador_navegacion_global/kinematics.py)): la idea de descomponer un movimiento en `(v, ω)` y convertirlo a velocidades de rueda es exactamente lo que usamos para ejecutar la ruta planificada.

Del Laboratorio 2 reutilizamos casi sin cambios la lectura de sensores de distancia, el filtrado EMA + Kalman y la navegación reactiva de evitación de obstáculos ([sensors.py](controllers/controlador_navegacion_global/sensors.py), [obstacle_avoidance.py](controllers/controlador_navegacion_global/obstacle_avoidance.py)). Ahora se combinan con la odometría, que alimenta tanto el filtro de Kalman como el seguidor de ruta.

Lo que agrega el proyecto final es la capa de navegación global, la grilla de ocupación y el A*, que en los laboratorios no existía: ahí la navegación era puramente reactiva y local.

## 7. Resultados y métricas

El controlador imprime por consola, en cada ejecución, el largo de la ruta planificada por A*, el tiempo hasta llegar a la meta, la distancia recorrida real (acumulada por odometría) y la pose estimada final.

| Métrica | Escenario simple | Escenario complejo |
|---|---|---|
| Tiempo hasta la meta (s) | 42.7 | 127.2 |
| Longitud ruta planificada (m) | 3.51 | 9.42 |
| Longitud trayectoria ejecutada (m) | 3.35 | 9.52 |
| Diferencia ruta vs. trayectoria (m) | -0.16 | +0.10 |
| N° de activaciones de evitación de obstáculos | 0 | |
| Colisiones | 0 | 0 |
| % de ejecuciones exitosas | 100% (3/3) | 100% (3/3) |

Corrimos cada escenario 3 veces desde las mismas condiciones iniciales y los 3 resultados fueron idénticos. Esto tiene sentido: el sistema es determinista en este entorno, no hay ruido en los sensores ni perturbaciones externas, así que las 3 corridas cuentan como 3/3 exitosas, no como un promedio de valores distintos.

El escenario complejo necesita una ruta casi 3 veces más larga (9.42 contra 3.51 m) y casi 3 veces más tiempo (127.2 contra 42.7 s) que el simple, lo cual tiene sentido porque hay que cruzar 5 corredores con puertas angostas en vez de solo rodear un obstáculo. Algo que nos llamó la atención: el signo de la diferencia entre ruta y trayectoria cambia entre escenarios. En el complejo la trayectoria ejecutada queda más larga que la planificada (+0.10 m), por las correcciones de la evitación reactiva al pasar las puertas estrechas. En el simple queda más corta (-0.16 m), porque el robot entra en la tolerancia de meta (`GOAL_TOLERANCE = 0.20 m`) antes de terminar el último tramo planificado y se detiene ahí.

Todavía no registramos `(x, y)` estimado en cada paso, solo la pose final, así que si queremos graficar ruta planificada contra trayectoria real falta agregar un logger CSV simple en el loop principal.

## 8. Capturas, gráficos y video

Capturas cenitales de ambos escenarios en la sección 4 (`Image/Imagen simple.png`, `Image/Imagen Complejo.png`).

Gráficos:

![Gráfico1](Image/Gráfico1.png)
Ruta planificada vs. trayectoria ejecutada.

![Gráfico2](Image/Gráfico2.png)
Tiempo hasta alcanzar la meta.

![Gráfico3](Image/Gráfico3.png)
Diferencia entre ruta planificada y ejecutada.

![Gráfico4](Image/Gráfico4.png)
Factor de escala entre el escenario complejo y el simple.

Video demostrativo, ejecución completa de inicio a meta:
- [Escenario simple](https://drive.google.com/file/d/1xoUUvC64dIvH_BtWeafztG8QDLntDhwk/view?usp=sharing)
- [Escenario complejo](https://drive.google.com/file/d/1btlqDcIMntdpaV1ImWIlnB6zs8sf2AmE/view?usp=sharing)

## 9. Instrucciones de ejecución

1. Abrir Webots (R2025a) y cargar `worlds/escenario_simple.wbt` o `worlds/escenario_complejo.wbt`.
2. El controlador `controlador_navegacion_global` ya está asignado al robot en cada mundo, con el nombre del escenario como argumento (`controllerArgs`).
3. Correr la simulación. Por consola se imprime la ruta calculada por A* y, al llegar a la meta, el resumen de la ejecución.
4. Para apuntar el controlador a otro escenario a mano, basta con cambiar el argumento en `controllerArgs` del nodo `E-puck` por `"escenario_simple"` o `"escenario_complejo"`.

## 10. Conclusiones, limitaciones y mejoras

El robot llega a la meta en los dos escenarios, sin chocar, siguiendo la ruta que calcula A* sobre la grilla de ocupación, y lo hace en el 100% de las corridas que hicimos (3 de 3 por escenario).

La odometría funciona bien en un entorno como Webots, donde no hay ruido real en los encoders ni perturbaciones externas: en los dos escenarios la pose estimada al llegar a la meta coincide con la posición real del robot. Pero igual es un método que acumula error con el tiempo, porque cada paso de integración arrastra el error del paso anterior, así que en una ruta más larga como la del escenario complejo (casi 9.5 m) uno esperaría más deriva que en una corta. En un robot físico real ese efecto se nota mucho antes, a los pocos metros, y sin alguna corrección externa el robot termina perdiendo la pista de dónde está. Eso sí, la diferencia que medimos entre ruta planificada y trayectoria (+0.10 m en el complejo, -0.16 m en el simple) no viene de deriva de odometría, sino de las correcciones que hace la evitación reactiva al cruzar las puertas angostas.

El otro punto que vale la pena anotar es el conflicto entre la evitación reactiva y la ruta planificada. Como armamos el sistema en dos niveles separados, cuando se activa la evitación toma el control completo del robot sin saber nada de la ruta global, y cuando se desactiva el seguidor de ruta retoma el mando. Esto se nota de dos formas durante las pruebas: al cruzar las puertas del escenario complejo, los sensores laterales a veces detectan las paredes y activan la evitación aunque el robot vaya bien encaminado, generando un retroceso o giro que lo aleja un poco del waypoint actual (de ahí salen los +0.10 m extra de trayectoria). Y después de una secuencia de BACKOFF, TURN y ESCAPE, el robot puede quedar mirando hacia un lado que no es el óptimo para el siguiente waypoint, así que el seguidor tiene que corregir con un giro de más, algo que vimos más seguido en el complejo que en el simple.

Para una próxima versión, lo primero que probaríamos es replanificar sobre la marcha: si la evitación se activa varias veces seguidas, volver a correr A* desde la pose actual en vez de seguir forcejeando en la misma zona. También nos gustaría sacar la conmutación binaria entre modo reactivo y modo global, y reemplazarla por algo tipo campo potencial o DWA que combine la atracción hacia el waypoint con la repulsión de obstáculos en una sola ley de control, así no hay pelea de prioridades entre los dos sistemas. Otra mejora posible es usar todos los sensores IR del e-puck (no solo los frontales y laterales que ya usamos) para tener percepción lateral completa, y sumar algún sensor de posición absoluta (un GPS simulado de Webots, o landmarks conocidos) para corregir la deriva odométrica de vez en cuando con un filtro de Kalman extendido. Y si en algún momento queremos ir más allá de planificación de rutas, una extensión natural sería construir la grilla de ocupación en tiempo real a partir de las lecturas IR y la odometría en vez de cargarla fija desde el escenario, que es básicamente la puerta de entrada a la Línea B.
