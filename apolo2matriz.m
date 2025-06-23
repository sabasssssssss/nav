function [fila, columna] = apolo2matriz(x_apolo, y_apolo)
    % Conversión de coordenadas del sistema Apolo (origen abajo derecha)
    % a índices de la matriz de ocupación (origen arriba izquierda).
    %  - x_apolo: Coordenada vertical en metros
    %  - y_apolo: Coordenada horizontal en metros
    % Devuelve [fila, columna] como índices 1-based del mapa binario.

    % Validar el rango de entrada. Se permiten valores dentro
    % del tamaño del mapa excluyendo el límite superior para
    % evitar celdas fuera de rango al convertir.
    if x_apolo < 0 || x_apolo >= 10 || y_apolo < 0 || y_apolo >= 33
        error('Coordenadas de entrada fuera de los límites permitidos.');
    end

    disp(['[DEBUG] Entrando a apolo2matriz con x_apolo=', num2str(x_apolo), ', y_apolo=', num2str(y_apolo)]);

    % Convertir usando floor para asegurar valores dentro de los 
    % límites de 1..10 y 1..33 respectivamente.
    fila    = 10 - floor(x_apolo);
    columna = 33 - floor(y_apolo);

    if fila < 1 || fila > 10 || columna < 1 || columna > 33
        error('Coordenadas convertidas fuera de los límites del mapa.');
    end

    disp(['[DEBUG] Salida de apolo2matriz: fila=', num2str(fila), ', columna=', num2str(columna)]);
end 