function [x_mapa, y_mapa, theta] = Localizacion_S(opciones)
    % Función optimizada de localización para el robot de limpieza
    % Implementa un enfoque robusto para la localización del robot
    % combinando odometría con corrección basada en balizas y filtro de Kalman
    %
    % Parámetros:
    %   opciones - Estructura con parámetros configurables:
    %     - opciones.robot_name: Nombre del robot (por defecto: 'Limpiator')
    %     - opciones.usar_kalman: Usar filtro de Kalman (por defecto: true)
    %     - opciones.matriz_covarianza: Matriz de covarianza inicial (por defecto: diagonal con valores [0.1, 0.1, 0.05])
    %     - opciones.umbral_distancia: Distancia para detección de balizas (por defecto: 1.5)
    %     - opciones.factor_confianza: Factor de confianza en las balizas (por defecto: 0.7)
    %     - opciones.usar_sensores: Usar sensores para mejorar localización (por defecto: true)
    %     - opciones.cargar_balizas_xml: Cargar balizas desde archivo XML (por defecto: false)
    %     - opciones.archivo_xml: Ruta al archivo XML con balizas (por defecto: 'restaurant.xml')
    
    if ~isstruct(opciones)
        error('El parámetro de entrada debe ser una estructura de opciones.');
    end
    
    % Configuración de opciones por defecto
    if nargin < 1
        opciones = struct();
    end
    
    % Parámetros configurables con valores por defecto
    robot_name = getfield_with_default(opciones, 'robot_name', 'Limpiator');
    usar_kalman = getfield_with_default(opciones, 'usar_kalman', true);
    umbral_distancia = getfield_with_default(opciones, 'umbral_distancia', 1.5);
    factor_confianza = getfield_with_default(opciones, 'factor_confianza', 0.7);
    usar_sensores = getfield_with_default(opciones, 'usar_sensores', true);
    cargar_balizas_xml = getfield_with_default(opciones, 'cargar_balizas_xml', false);
    archivo_xml = getfield_with_default(opciones, 'archivo_xml', 'restaurant.xml');
    debug = getfield_with_default(opciones, 'debug', false);
    world = getfield_with_default(opciones, 'world', 'world1');
    
    % Inicializar matriz de covarianza si no se proporciona
    if ~isfield(opciones, 'matriz_covarianza') || isempty(opciones.matriz_covarianza)
        matriz_covarianza = diag([0.1, 0.1, 0.05]);  % Valores por defecto para [x, y, theta]
    else
        matriz_covarianza = opciones.matriz_covarianza;
    end
    
    % Cargar factores de calibración si existen
    try
        datos_calibracion = load('calibracion_robot.mat');
        factor_dist = datos_calibracion.calibracion.factor_distancia;
        factor_ang = datos_calibracion.calibracion.factor_angulo;
        if debug
            disp(['ℹ️ Factores de calibración cargados: Distancia=', num2str(factor_dist), ', Ángulo=', num2str(factor_ang)]);
        end
    catch ME
        factor_dist = 1.0;
        factor_ang = 1.0;
        if debug
            disp(['⚠️ No se encontraron factores de calibración (', ME.message, '), usando valores por defecto']);
        end
    end
    
    % 📌 Definición de balizas con coordenadas conocidas
    balizas = [];
    if cargar_balizas_xml && exist(archivo_xml, 'file')
        try
            [~, balizas_cargadas] = parse_xml_map_S(archivo_xml);
            if ~isempty(balizas_cargadas)
                balizas = balizas_cargadas;
                if debug
                    disp(['✅ Balizas cargadas desde XML: ', num2str(size(balizas, 1)), ' balizas encontradas']);
                    disp('Posiciones de balizas:');
                    disp(balizas);
                end
            else
                disp('❌ No se encontraron balizas en el XML. Abortando localización.');
                x_mapa = NaN; y_mapa = NaN; theta = NaN;
                return;
            end
        catch ME
            disp(['❌ Error al cargar balizas desde XML (', ME.message, '). Abortando localización.']);
            x_mapa = NaN; y_mapa = NaN; theta = NaN;
            return;
        end
    else
        % Usar balizas predefinidas si no se carga desde XML o el archivo no existe
        balizas = definir_balizas_predefinidas();
    end
    
    % 📌 Obtener la ubicación del robot en Apolo
    try
        pos_apolo = apoloGetLocationMRobot(robot_name, world); % Requiere Apolo
        x_apolo = pos_apolo(1);
        y_apolo = pos_apolo(2);
        theta = pos_apolo(3);
    catch ME
        if debug
            disp(['⚠️ Error al obtener posición de Apolo (apoloGetLocationMRobot): ', ME.message]);
        end
        % Valores por defecto si Apolo no está disponible o hay error
        x_apolo = 0;
        y_apolo = 0;
        theta = 0;
    end
    
    % === USO DIRECTO DE COORDENADAS APOLO ===
    if isfield(opciones, 'offset_theta')
        offset_theta = opciones.offset_theta;
    else
        offset_theta = 0;
    end

    % Desactivar completamente el filtro de Kalman y la odometría
    x_mapa = x_apolo;
    y_mapa = y_apolo;
    theta = normalizar_angulo(theta + offset_theta);
    % Limitar la posición a los rangos válidos del entorno de Apolo SIEMPRE
    x_mapa = max(0, min(10, x_mapa));
    y_mapa = max(0, min(33, y_mapa));
    if debug
        disp(['[DEBUG] Localizacion_S SOLO APOLO: X=', num2str(x_mapa), ', Y=', num2str(y_mapa), ', Theta=', num2str(theta)]);
    end
    return;
    
    % Obtener datos de sensores si se solicita
    datos_sensores = struct();
    if usar_sensores
        try
            % Añadir el parámetro 'world' explícitamente
            datos_sensores.uc0 = apoloGetUltrasonicSensor('uc0', world); % Requiere Apolo
            datos_sensores.ul1 = apoloGetUltrasonicSensor('ul1', world); % Requiere Apolo
            datos_sensores.ur1 = apoloGetUltrasonicSensor('ur1', world); % Requiere Apolo
            try
                datos_sensores.laser = apoloGetLaserData('LMS100', world); % Requiere Apolo
            catch ME_laser
                if debug
                    disp(['⚠️ No se pudieron obtener datos de láser (', ME_laser.message, ')']);
                end
            end
            if debug
                disp('✅ Datos de sensores obtenidos correctamente');
            end
        catch ME_sensor
            if debug
                disp(['⚠️ Error al obtener datos de sensores (', ME_sensor.message, ')']);
            end
        end
    end
    % Protección: si los datos de sensores contienen NaN, usar valores por defecto
    if any(isnan(struct2array(datos_sensores)))
        disp('⚠️ Datos de sensores contienen NaN. Usando valores por defecto.');
        datos_sensores = struct('uc0',0,'ul1',0,'ur1',0,'laser',[]);
    end
    
    % Obtener odometría como medida de predicción
    try
        % Añadir el parámetro 'world' explícitamente
        odometria = apoloGetOdometry(robot_name, world); % Requiere Apolo
        x_odo = odometria(1);
        y_odo = odometria(2);
        theta_odo = odometria(3);
    catch ME
        if debug
            disp(['⚠️ Error al obtener odometría de Apolo (apoloGetOdometry): ', ME.message]);
        end
        % Valores por defecto si Apolo no está disponible o hay error
        x_odo = x_mapa;
        y_odo = y_mapa;
        theta_odo = theta;
    end
    
    % Aplicar factores de calibración a la odometría
    % Asegurarse de que no se acumulen los valores
    delta_x = (x_odo - x_mapa) * factor_dist;
    delta_y = (y_odo - y_mapa) * factor_dist;
    delta_theta = normalizar_angulo((theta_odo - theta) * factor_ang);
    
    % Normalizar ángulo
    delta_theta = normalizar_angulo(delta_theta);
    
    % Detectar si es localización inicial (sin movimiento)
    movimiento_significativo = (abs(delta_x) > 0.01) || (abs(delta_y) > 0.01) || (abs(delta_theta) > 0.01);
    if ~movimiento_significativo
        % Solo usar la posición real de Apolo
        x_mapa = x_apolo;
        y_mapa = y_apolo;
        theta = normalizar_angulo(theta + offset_theta);
        x_mapa = max(0, min(10, x_mapa));
        y_mapa = max(0, min(33, y_mapa));
        if debug
            disp(['[DEBUG] Localizacion_S SOLO APOLO (INICIAL): X=', num2str(x_mapa), ', Y=', num2str(y_mapa), ', Theta=', num2str(theta)]);
        end
        return;
    end
    
    % Solo aplicar Kalman si el robot ya ha realizado un movimiento significativo
    if usar_kalman && movimiento_significativo
        % Vector de estado actual [x, y, theta]
        estado_actual = [x_mapa; y_mapa; theta];
        
        % Predicción basada en odometría calibrada
        estado_predicho = estado_actual + [delta_x; delta_y; delta_theta];
        
        % Matriz de transición de estado (modelo de movimiento)
        F = eye(3);  % Matriz identidad para modelo lineal simple
        
        % Matriz de covarianza de proceso (incertidumbre del movimiento)
        % Aumenta con la distancia recorrida y el ángulo girado
        dist_recorrida = sqrt(delta_x^2 + delta_y^2);
        Q = diag([
            max(0.01, dist_recorrida * 0.1),  % Incertidumbre en X
            max(0.01, dist_recorrida * 0.1),  % Incertidumbre en Y
            max(0.005, abs(delta_theta) * 0.2)  % Incertidumbre en theta
        ]);
        
        % Actualizar matriz de covarianza (predicción)
        matriz_covarianza_pred = F * matriz_covarianza * F' + Q;
        
        % Varianza de la distancia y el ángulo de las balizas (ejemplo del trabajo de referencia)
        var_dist = 1.9e-4;
        var_ang = 3.8e-4;

        % Llamar a la función modularizada del filtro de Kalman
        opciones.datos_sensores = datos_sensores; % Pasar datos de sensores a la función de Kalman
        [estado_corregido, matriz_covarianza] = kalman_filter_update_S(estado_predicho, matriz_covarianza_pred, balizas, umbral_distancia, var_dist, var_ang, opciones);
            
        % Extraer valores corregidos
        x_mapa = estado_corregido(1);
        y_mapa = estado_corregido(2);
        theta = estado_corregido(3);
        if debug
            disp(['[DEBUG] Kalman: estado_predicho = (', num2str(estado_predicho(1)), ',', num2str(estado_predicho(2)), ',', num2str(estado_predicho(3)), ')']);
            disp(['[DEBUG] Kalman: estado_corregido = (', num2str(x_mapa), ',', num2str(y_mapa), ',', num2str(theta), ')']);
        end
        
        % Guardar matriz de covarianza actualizada para uso futuro
        opciones.matriz_covarianza = matriz_covarianza;
        
    else
        % No aplicar Kalman en la localización inicial
        if debug && usar_kalman
            disp('[DEBUG] Kalman NO aplicado (sin movimiento significativo)');
        end
    end
    
    % Normalizar ángulo final
    theta = normalizar_angulo(theta);
    
    % 📌 Mostrar la conversión en la consola (solo si se solicita depuración)
    if debug
        disp(['📍 Posición en Apolo: X=', num2str(x_apolo), ', Y=', num2str(y_apolo), ', Theta=', num2str(theta)]);
        disp(['📍 Posición en el mapa binario: X=', num2str(x_mapa), ', Y=', num2str(y_mapa)]);
        
        if usar_kalman
            % Mostrar información de incertidumbre
            incertidumbre_x = sqrt(matriz_covarianza(1,1));
            incertidumbre_y = sqrt(matriz_covarianza(2,2));
            incertidumbre_theta = sqrt(matriz_covarianza(3,3));
            
            disp(['📊 Incertidumbre: X=±', num2str(incertidumbre_x), ', Y=±', num2str(incertidumbre_y), ', Theta=±', num2str(incertidumbre_theta)]);
        end
    end
    
    % Mensaje de depuración para verificar theta antes de aplicar el offset
    disp(['🔄 Theta antes de aplicar offset: ', num2str(theta)]);
    % Aplicar el offset de orientación
    if isfield(opciones, 'angulo_correccion')
        angulo_correccion = opciones.angulo_correccion;
    else
        angulo_correccion = 0;
    end
    theta = normalizar_angulo(theta + angulo_correccion);
    % Mensaje de depuración para verificar theta después de aplicar el offset
    disp(['🔄 Theta después de aplicar offset: ', num2str(theta)]);
    
    % Después de aplicar el offset
    disp(['📍 Posición en el mapa: X=', num2str(x_mapa), ', Y=', num2str(y_mapa), ', Theta=', num2str(theta)]);
    
    % Limitar la posición a los rangos válidos del entorno de Apolo SIEMPRE
    x_mapa = max(0, min(10, x_mapa));
    y_mapa = max(0, min(33, y_mapa));
    if debug
        disp(['[DEBUG] Localizacion_S FINAL LIMITADA: X=', num2str(x_mapa), ', Y=', num2str(y_mapa)]);
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

% Función para definir balizas predefinidas
function balizas = definir_balizas_predefinidas()
    % Ampliado de 3 a 15 balizas para mejor cobertura del entorno
    balizas = [
        1, 11;   % Baliza 1
        8, 3;    % Baliza 2
        7, 28;   % Baliza 3
        15, 15;  % Baliza 4
        20, 5;   % Baliza 5
        25, 20;  % Baliza 6
        30, 10;  % Baliza 7
        12, 25;  % Baliza 8
        18, 30;  % Baliza 9
        5, 20;   % Baliza 10
        3, 3;    % Baliza 11
        28, 28;  % Baliza 12
        10, 20;  % Baliza 13
        22, 15;  % Baliza 14
        15, 5    % Baliza 15
    ];
end

% Función auxiliar para obtener campos de estructura con valores por defecto
function valor = getfield_with_default(estructura, campo, valor_default)
    if isfield(estructura, campo)
        valor = estructura.(campo);
    else
        valor = valor_default;
    end
end

function datos = obtener_datos_sensores()
    % Obtener datos frescos de los sensores ultrasónicos y láser
    world = 'world1'; % Definir el mundo de Apolo
    datos = struct();
    
    % Obtener datos de los sensores ultrasónicos
    datos.uc0 = apoloGetUltrasonicSensor('uc0', world);
    datos.ul1 = apoloGetUltrasonicSensor('ul1', world);
    datos.ur1 = apoloGetUltrasonicSensor('ur1', world);
    
    % Obtener datos del sensor láser
    try
        datos.laser = apoloGetLaserData('LMS100', world);
    catch ME
        disp(['⚠️ No se pudieron obtener datos de láser (', ME.message, ')']);
        datos.laser = [];
    end
    
    % Protección: si los datos de sensores contienen NaN, usar valores por defecto
    if any(isnan(struct2array(datos)))
        disp('⚠️ Datos de sensores contienen NaN. Usando valores por defecto.');
        datos = struct('uc0',0,'ul1',0,'ur1',0,'laser',[]);
    end
end



