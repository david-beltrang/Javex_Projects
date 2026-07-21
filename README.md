# Javex_Projects — VEX Robotics Competition Code

Código fuente de robots VEX V5 para dos temporadas de competencia,
desarrollado por el equipo Javex de la Pontificia Universidad Javeriana,
clasificatorio nacional al mundial.

## Temporadas

| Temporada | Juego | Periodo | README |
|-----------|-------|---------|--------|
| High Stakes | VEX V5RC High Stakes | 2024-2025 | [High Stakes 2024-2025/README.md](High%20Stakes%202024-2025/README.md) |
| Push Back | VEX V5RC Push Back | 2025-2026 | [Push Back 2025-2026/README.md](Push%20Back%202025-2026/README.md) |

## Proyectos destacados

- **High Stakes**: `Grande_Azul` es el proyecto más modular y completo de la temporada (4 headers, 6 fases autónomas, 2 modos de driver). Autoría: Kenneth Bustamante.
- **Push Back**: `Estrategia1Izq` es el proyecto más completo (470 líneas en funciones.h, voltaje PID, 5 modos de recolección, 2 neumáticas). `implementacionIMU` tiene la mejor arquitectura (clase C++ PIDController con anti-windup y camino circular más corto).

## Toolchain

Los proyectos usan VEXcode Pro V5 (mayoría) o PROS CLI (proyectos con GPS/IMU nativo). Cada README de temporada tiene instrucciones de compilación específicas.

## Estado de testing

No hay testing automatizado en ningún proyecto. El código se validó en robot físico durante prácticas y competencias.

## Autor

David Beltran Gomez — VEX Robotics Competition, clasificatorio nacional a mundial
