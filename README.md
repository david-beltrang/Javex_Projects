# Javex_Projects — VEX Robotics Competition Code

Robots VEX V5 programados para dos temporadas de competencia en la
categoría **VEX U (University)** con el equipo Javex (Pontificia
Universidad Javeriana), clasificatorio nacional al mundial. Desde un
autónomo sin sensores escrito en mi primera semana en el equipo, hasta
un controlador PID en C++ con anti-windup y corrección de rumbo por IMU.

## Temporadas

| Temporada | Juego | Periodo | Detalle |
|-----------|-------|---------|---------|
| **High Stakes** | Anotar Rings en Stakes, mover Mobile Goals, trepar el Ladder | 2024-2025 | [Ver proyectos →](High%20Stakes%202024-2025/README.md) |
| **Push Back** | Anotar Blocks en Goals, controlar Zones, Park final | 2025-2026 | [Ver proyectos →](Push%20Back%202025-2026/README.md) |

## Lo más interesante de cada temporada

**High Stakes** — `Grande_Azul` corre una rutina autónoma de 6 fases (anotar
anillo, recuperar meta móvil, recolectar anillos superiores e inferiores,
anillos de esquina, acercarse a la escalera), con 8 funciones de
movimiento distintas para curvas y rotaciones. Autoría: Kenneth Bustamante.

**Push Back** — `implementacionIMU` reemplaza el control proporcional
simple por una clase `PIDController` en C++ con tres instancias
independientes (giro, avance, corrección de rumbo), anti-windup, y
cálculo de camino circular más corto para rotaciones. `Estrategia1Izq`
es el proyecto de competencia real: 470 líneas de lógica de recolección
con 5 sub-modos distintos y control por voltaje.

También hay dos experimentos con navegación GPS por waypoints
(`pruebas-GPS-CVex`, `pruebas-GPS-PROS`) usando `atan2` para corrección
de rumbo hacia una coordenada absoluta del campo.

## De dónde vengo a dónde llegué

El primer proyecto del repositorio (`My_First_Autonomous`, mi prueba de
admisión al equipo) mueve el robot con secuencias puramente
temporizadas, sin un solo sensor. El último (`implementacionIMU`) usa
retroalimentación de IMU, tres controladores PID independientes, y
saturación de voltaje. Cada README de temporada documenta esa
progresión proyecto por proyecto, con código real, no solo la versión
final.

## Toolchain

VEXcode Pro V5 (mayoría de proyectos) o PROS CLI (proyectos con acceso
nativo a GPS/IMU). Instrucciones de compilación específicas en cada
README de temporada.

## Estado de testing

Sin testing automatizado — validación en robot físico durante
prácticas y competencia, como es estándar en este tipo de desarrollo.

## Equipo de programación

| Temporada | Programadores |
|-----------|----------------|
| High Stakes 2024-2025 | Kenneth Bustamante, Melisa Ruiz Barrera, **David Beltrán Gómez** |
| Push Back 2025-2026 | Melisa Ruiz, Miguel Casallas, **David Beltrán Gómez** |

Este repositorio contiene el código commiteado por David Beltrán Gómez;
la autoría individual de cada proyecto (cuando se pudo verificar por
git log o comentarios en el código) está documentada en el README de
cada temporada.

## Autor

David Beltrán Gómez — VEX Robotics Competition, categoría VEX U,
clasificatorio nacional a mundial
