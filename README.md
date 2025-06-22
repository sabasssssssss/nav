# Proyecto de Navegación Robótica

Este proyecto implementa un sistema de navegación robótica utilizando MATLAB. El sistema se encarga de la localización, planificación de rutas y detección de obstáculos en un entorno simulado.

## Estructura del Proyecto

- **apolo2matriz.m**: Convierte coordenadas del sistema Apolo a índices de matriz.
- **matriz2apolo.m**: Convierte índices de matriz a coordenadas del sistema Apolo.
- **navegador_principal_S.m**: Función principal que coordina la navegación del robot.
- **obstaculos_S.m**: Verifica y ajusta la ruta en presencia de obstáculos.
- **mensajero_exploracion_S.m**: Planifica una ruta de exploración utilizando el algoritmo A*.

## Requisitos

- MATLAB R2020 o superior.
- Toolbox de Simulink (opcional, dependiendo de la implementación).

## Instalación

1. Clona este repositorio en tu máquina local:
   ```bash
   git clone <URL_DEL_REPOSITORIO>
   ```
2. Abre MATLAB y navega a la carpeta del proyecto.
3. Ejecuta `navegador_principal` para iniciar el sistema de navegación.

## Uso

Ajusta las coordenadas iniciales y los objetivos en el archivo `navegador_principal_S.m` según sea necesario. Ejecuta el script para comenzar la simulación.

## Contribuciones

Las contribuciones son bienvenidas. Por favor, abre un issue o envía un pull request para discutir cambios.

## Licencia

Este proyecto está bajo la Licencia MIT. 