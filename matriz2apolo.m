function [x_apolo, y_apolo] = matriz2apolo(fila, columna)
    % Conversión inversa de coordenadas del mapa de MATLAB al sistema Apolo.
    %  - fila:    Índice de fila en el mapa (1..10)
    %  - columna: Índice de columna en el mapa (1..33)
    % Devuelve las coordenadas [x_apolo, y_apolo] en metros para el simulador.

    % Validar entrada
    if fila < 1 || fila > 10 || columna < 1 || columna > 33
        error('Coordenadas convertidas fuera de los límites del mapa.');
    end

    % Aplicar la transformación inversa
    x_apolo = 10 - fila;  % Conversión de fila a coordenada Apolo
    y_apolo = 33 - columna; % Conversión de columna a coordenada Apolo

    % Verificación adicional para evitar valores fuera de rango
    if x_apolo < 0 || x_apolo > 10 || y_apolo < 0 || y_apolo > 33
        error('Coordenadas resultantes fuera de los límites permitidos.');
    end
<<<<<<< HEAD
end
=======
end
>>>>>>> cccd5430266764927947a06de4fe1bc4c4dcf368
