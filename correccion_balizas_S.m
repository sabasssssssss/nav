function correccion_balizas_S(opciones)
    % Función para la corrección de la odometría basada en balizas y datos de sensores.
    % Utiliza las balizas cargadas desde el XML para refinar la pose del robot.
    %
    % Parámetros:
    %   opciones - Estructura con parámetros configurables:
    %     - opciones.robot_name: Nombre del robot (por defecto: 'Limpiator')
    %     - opciones.debug: Mostrar mensajes de depuración (por defecto: false)
    %     - opciones.archivo_xml: Ruta al archivo XML con balizas (por defecto: 'root1.xml')
    %     - opciones.world: Nombre del mundo de Apolo
    %     - opciones.mapa_actual: El mapa de ocupación actual (matriz lógica)
    %     - opciones.info_mapa: Estructura con la información del mapa (resolucion, min_apolo_x, etc.)
    
    robot_name = getfield_with_default(opciones, 'robot_name', 'Limpiator');
    debug = getfield_with_default(opciones, 'debug', false);
    archivo_xml = getfield_with_default(opciones, 'archivo_xml', 'root1.xml'); % Usar root1.xml
    world = getfield_with_default(opciones, 'world', 'world2');
    mapa = getfield_with_default(opciones, 'mapa_actual', []);
    info_mapa = getfield_with_default(opciones, 'info_mapa', []);

    if debug
        disp('ℹ️ Realizando corrección de balizas...');
    end

    % Cargar balizas desde el XML especificado
    balizas_conocidas = [];
    if exist(archivo_xml, 'file')
        try
            [~, balizas_cargadas] = parse_xml_map_S(archivo_xml);
            if ~isempty(balizas_cargadas)
                balizas_conocidas = balizas_cargadas;
                if debug
                    disp(['✅ Balizas cargadas desde XML para corrección: ', num2str(size(balizas_conocidas, 1)), ' balizas.']);
                    disp('Posiciones de balizas:');
                    disp(balizas_conocidas);
                end
            else
                disp('❌ No se encontraron balizas en el XML para corrección. Abortando corrección.');
                return;
            end
        catch ME
            disp(['❌ Error al cargar balizas desde XML para corrección (', ME.message, '). Abortando corrección.']);
            return;
        end
    else
        if debug
            disp(['⚠️ Archivo XML de balizas no encontrado: ', archivo_xml, '. No se realizará corrección basada en balizas.']);
        end
    end

    % Obtener la posición actual del robot (simulada o real de Apolo)
    try
        pos_apolo = apoloGetLocationMRobot(robot_name, world);
        x_actual = pos_apolo(1);
        y_actual = pos_apolo(2);
        theta_actual = pos_apolo(3);
    catch ME
        if debug
            disp(['⚠️ No se pudo obtener la posición de Apolo para corrección: ', ME.message]);
        end
        if debug
            disp('❌ No se puede realizar corrección de balizas sin datos de Apolo.');
        end
        return;
    end

    % Simular la detección de balizas cercanas y aplicar una corrección
    if ~isempty(balizas_conocidas)
        distancias_a_balizas = sqrt((balizas_conocidas(:,1) - x_actual).^2 + (balizas_conocidas(:,2) - y_actual).^2);
        [min_dist, idx_baliza_cercana] = min(distancias_a_balizas);

        umbral_deteccion = 2.0; % Distancia máxima para considerar una baliza detectada

        if min_dist < umbral_deteccion
            baliza_detectada = balizas_conocidas(idx_baliza_cercana,:);
            if debug
                disp(['📡 Baliza detectada cerca en (', num2str(baliza_detectada(1)), ',', num2str(baliza_detectada(2)), ')']);
            end

            % Calcular la corrección necesaria (diferencia entre la posición estimada y la baliza)
            correccion_x = baliza_detectada(1) - x_actual;
            correccion_y = baliza_detectada(2) - y_actual;
            
            % Aplicar una fracción de la corrección para evitar saltos bruscos
            factor_correccion = 0.5; % Ajustar este factor según la confianza en la baliza
            x_corregido = x_actual + correccion_x * factor_correccion;
            y_corregido = y_actual + correccion_y * factor_correccion;
            theta_corregido = theta_actual; % Asumimos que la baliza no corrige el ángulo directamente

            try
                apoloSetRobotPose_S(robot_name, x_corregido, y_corregido, theta_corregido, world);
                apoloUpdate(world);
                if debug
                    disp(['✅ Posición corregida en Apolo a (', num2str(x_corregido), ',', num2str(y_corregido), ',', num2str(theta_corregido), ')']);
                end

                % === Actualizar el mapa de ocupación con la nueva posición del robot ===
                % Marcar la celda actual del robot como libre (si no es un obstáculo permanente)
                if ~isempty(mapa) && ~isempty(info_mapa)
                    % Usar convención global
                    [fila, columna] = apolo2matriz(x_corregido, y_corregido);
                    [mapa_filas, mapa_cols] = size(mapa);
                    if columna >= 1 && columna <= mapa_cols && fila >= 1 && fila <= mapa_filas
                        mapa(fila, columna) = true;
                        if debug
                            disp(['✅ Celda del robot (fila=', num2str(fila), ', columna=', num2str(columna), ') marcada como libre en el mapa.']);
                        end
                    else
                        if debug
                            disp(['⚠️ Celda del robot fuera de los límites del mapa (fila=', num2str(fila), ', columna=', num2str(columna), ').']);
                            disp(['[DEBUG] Límites: filas 1-', num2str(mapa_filas), ', columnas 1-', num2str(mapa_cols)]);
                        end
                    end
                end

            catch ME
                if debug
                    disp(['⚠️ Error al aplicar corrección en Apolo: ', ME.message]);
                end
            end
        else
            if debug
                disp('ℹ️ No se detectaron balizas cercanas para corrección.');
            end
        end
    else
        if debug
            disp('ℹ️ No hay balizas conocidas para realizar corrección.');
        end
    end
end

% Función auxiliar para obtener campos de estructura con valores por defecto
function valor = getfield_with_default(estructura, campo, valor_default)
    if isfield(estructura, campo)
        valor = estructura.(campo);
    else
        valor = valor_default;
    end
end

% Función para normalizar ángulos al rango [-pi, pi]
function angulo = normalizar_angulo(angulo)
    while angulo > pi
        angulo = angulo - 2*pi;
    end
    while angulo < -pi
        angulo = angulo + 2*pi;
    end
end

function [fila, columna] = apolo2matriz(x_apolo, y_apolo)
    fila    = 10 - x_apolo;
    columna = 33 - y_apolo;
    fila = round(fila);
    columna = round(columna);
end


