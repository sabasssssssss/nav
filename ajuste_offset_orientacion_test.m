% === Script de ajuste interactivo de offset y orientación mapa-Apolo ===
% Ahora usa resolución y offsets del archivo .mat generado

clear; clc;

% Carga el mapa lógico y la info de referencia
load('mapa_ocupacion.mat', 'mapa', 'info_mapa');
resolucion = info_mapa.resolucion;
offset_x = info_mapa.offset_apolo_x_a_matlab_y;
offset_y = info_mapa.offset_apolo_y_a_matlab_x;

% === PARÁMETROS DE AJUSTE (puedes modificar solo la rotación) ===
rotacion_mapa_deg = 0; % Rotación del mapa respecto a Apolo (grados, antihorario)
rotacion_mapa_rad = deg2rad(rotacion_mapa_deg);

% === PUNTOS DE PRUEBA EN APOLO/XML (en metros) ===
puntos_apolo = [
    0 0;
    0 5;
    5 0;
    5 5;
    10 10
];

% === VISUALIZACIÓN ===
figure; imshow(~mapa, 'InitialMagnification', 'fit'); colormap(gray); hold on;
for i = 1:size(puntos_apolo,1)
    [fila, columna] = apolo2matriz(puntos_apolo(i,2), puntos_apolo(i,1));
    if fila >= 1 && fila <= size(mapa,1) && columna >= 1 && columna <= size(mapa,2)
        scatter(columna, fila, 100, 'g', 'filled');
        text(columna+1, fila, sprintf('(%g,%g)', puntos_apolo(i,1), puntos_apolo(i,2)), 'Color', 'yellow');
    else
        disp(['⚠️ Punto Apolo fuera de límites: fila=', num2str(fila), ', columna=', num2str(columna)]);
    end
end

% === VISUALIZACIÓN DE BALIZAS DESDE XML ===
xml_path = 'root1.xml';
[~, balizas_xml] = parse_xml_map_S(xml_path);
for i = 1:size(balizas_xml,1)
    [fila_b, columna_b] = apolo2matriz(balizas_xml(i,2), balizas_xml(i,1));
    if fila_b >= 1 && fila_b <= size(mapa,1) && columna_b >= 1 && columna_b <= size(mapa,2)
        scatter(columna_b, fila_b, 80, 'r', 'filled');
        text(columna_b+1, fila_b, sprintf('Baliza(%g,%g)', balizas_xml(i,1), balizas_xml(i,2)), 'Color', 'red');
    else
        disp(['⚠️ Baliza fuera de límites: fila=', num2str(fila_b), ', columna=', num2str(columna_b)]);
    end
end

disp('Haz clic en el mapa para ver la posición equivalente en Apolo/XML. Pulsa Enter para terminar.');
while true
    [x_map, y_map, btn] = ginput(1);
    if isempty(x_map) || isempty(y_map) || btn ~= 1
        break;
    end
    [fila_c, columna_c] = apolo2matriz(y_map, x_map);
    [x_apolo, y_apolo] = matriz2apolo(fila_c, columna_c);
    disp(['[MAPA->APOLO] Celda (', num2str(x_map), ',', num2str(y_map), ') => Apolo (', num2str(x_apolo), ',', num2str(y_apolo), ')']);
    scatter(x_map, y_map, 120, 'b', 'o', 'LineWidth', 2);
    text(x_map+2, y_map, sprintf('A:(%.2f,%.2f)', x_apolo, y_apolo), 'Color', 'blue');
end

title('Ajuste de Offset y Orientación (puntos Apolo y balizas sobre mapa MATLAB)');
hold off;

disp('Modifica rotacion_mapa_deg y vuelve a ejecutar para ajustar.');

% Las funciones de conversión se utilizan ahora desde los archivos compartidos
% apolo2matriz.m y matriz2apolo.m para coincidir con el resto del código.
