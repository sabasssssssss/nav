x_apolo = 10 - fila;  % Conversión de fila a coordenada Apolo
if x_apolo < 0 || x_apolo > 10
    error('Coordenadas convertidas fuera de los límites del mapa.');
end

y_apolo = 33 - columna; % Conversión de columna a coordenada Apolo
if y_apolo < 0 || y_apolo > 33
    error('Coordenadas convertidas fuera de los límites del mapa.');
end 