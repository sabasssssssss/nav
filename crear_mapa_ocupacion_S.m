function [mapa, info_mapa] = crear_mapa_ocupacion_S()
    % Función simplificada para crear un mapa de ocupación binario.
    % Devuelve:
    %   mapa: matriz lógica (true=libre, false=ocupado)
    %   info_mapa: struct con resolucion, offset_x, offset_y, min_x, min_y, tamano
    
    % Coordenadas fijas de los obstáculos y balizas
    vertices_obstaculos = [0, 0; 1, 0; 1, 1; 0, 1]; % Ejemplo de obstáculos
    balizas_xml = [0.5, 0.5]; % Ejemplo de baliza
    
    % Definir resolución y offsets
    resolucion = 1;
    offset_apolo_y_a_matlab_x = 0;
    offset_apolo_x_a_matlab_y = 0;
    
    % Tamaño del mapa en celdas
    num_filas_mapa = 10;
    num_cols_mapa = 33;
    
    % Crear mapa lógico (true=libre)
    mapa_logico = true(num_filas_mapa, num_cols_mapa);
    
    % Añadir obstáculos
    for k = 1:size(vertices_obstaculos,1)
        x_celda_matlab = round(vertices_obstaculos(k,1) * resolucion) + 1;
        y_celda_matlab = round(vertices_obstaculos(k,2) * resolucion) + 1;
        mapa_logico(y_celda_matlab, x_celda_matlab) = false; 
    end
    
    % Añadir balizas (como libres)
    for i = 1:size(balizas_xml,1)
        x_celda_matlab = round(balizas_xml(i,1) * resolucion) + 1;
        y_celda_matlab = round(balizas_xml(i,2) * resolucion) + 1;
        mapa_logico(y_celda_matlab, x_celda_matlab) = true; 
    end
    
    mapa = mapa_logico;
    info_mapa = struct('resolucion', resolucion, 'offset_apolo_x_a_matlab_y', offset_apolo_x_a_matlab_y, 'offset_apolo_y_a_matlab_x', offset_apolo_y_a_matlab_x, 'tamano_filas_matlab', num_filas_mapa, 'tamano_cols_matlab', num_cols_mapa);
end

function valor = getfield_with_default(estructura, campo, valor_default)
    if isfield(estructura, campo)
        valor = estructura.(campo);
    else
        valor = valor_default;
    end
end


