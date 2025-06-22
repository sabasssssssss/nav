%% Script de Prueba para el Sistema de Navegación Robótica
% Este script inicializa y ejecuta el sistema de navegación robótica integrado.
% Asegúrese de que todos los archivos .m y .xml necesarios estén en el mismo directorio
% o en el path de MATLAB.

clear;
clc;
close all;

%% Configuración del Entorno
% Asegúrese de que el simulador Apolo esté corriendo y configurado correctamente.
% Las funciones de Apolo (apoloMoveMRobot, apoloUpdate, etc.) son llamadas
% directamente por el sistema. Si Apolo no está disponible, el sistema
% intentará manejar los errores, pero la visualización y el control real
% del robot no funcionarán.

% Añadir el directorio actual al path de MATLAB para asegurar que todas las funciones
% sean encontradas.
addpath(genpath(pwd));

%% Ejecución del Sistema de Navegación
% Llamada a la función principal del sistema.
% Esta función orquesta la calibración, localización, planificación de ruta
% y ejecución de movimientos.

navegador_principal_S();

disp("\n--- Ejecución del script de prueba finalizada ---");
disp("Revise la ventana de comandos de MATLAB para los mensajes del sistema.");
disp("Si Apolo está conectado, observe el comportamiento del robot en el simulador.");


