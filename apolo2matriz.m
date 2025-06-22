function [fila, columna] = apolo2matriz(x_apolo, y_apolo)
    if x_apolo < 0 || x_apolo > 10 || y_apolo < 0 || y_apolo > 33
        error('Coordenadas de entrada fuera de los límites permitidos.');
    end

    disp(['[DEBUG] Entrando a apolo2matriz con x_apolo=', num2str(x_apolo), ', y_apolo=', num2str(y_apolo)]);
    fila    = 10 - x_apolo;
    columna = 33 - y_apolo;
    fila = round(fila);
    columna = round(columna);

    if fila < 1 || fila > 10 || columna < 1 || columna > 33
        error('Coordenadas convertidas fuera de los límites del mapa.');
    end

    disp(['[DEBUG] Salida de apolo2matriz: fila=', num2str(fila), ', columna=', num2str(columna)]);
end 