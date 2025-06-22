function navegador_principal
    % Función principal optimizada que integra todos los componentes del sistema de navegación
    % Coordina la calibración, localización, planificación y ejecución de movimientos
    
    disp("=== Iniciando Sistema de Navegación Avanzado ===");
    
    % Configuración general
    opciones = struct();
    opciones.robot_name = 'Limpiator';
    opciones.debug = true;
    opciones.usar_kalman = true;
    opciones.visualizar = true;
    opciones.cargar_balizas_xml = true; % Asegurarse de que las balizas se carguen desde XML
    opciones.archivo_xml = 'root1.xml'; % Usar root1.xml por defecto
    % === Cambia aquí el mundo que quieres usar ===
    opciones.world = 'world2'; % O 'world2' o 'world4'
    
    % === TELETRANSPORTE INICIAL DEL ROBOT EN APOLO ===
    % Define aquí la posición inicial deseada en Apolo (en metros y radianes)
    x_inicial_apolo = 0; % vertical (hacia arriba)
    y_inicial_apolo = 11; % horizontal (derecha a izquierda)
    theta_inicial_apolo = 0; % orientación en radianes
    world_apolo = 'world2'; % nombre exacto del mundo en Apolo

    % === CHEQUEO DE EXISTENCIA DE MUNDO Y ROBOT EN APOLO ===
    try
        pos = apoloGetLocationMRobot('Limpiator', world_apolo);
        disp(['[CHEQUEO] Robot Limpiator encontrado en el mundo ', world_apolo, '. Posición: X=', num2str(pos(1)), ', Y=', num2str(pos(2)), ', Theta=', num2str(pos(3))]);
    catch
        error('[CHEQUEO] El robot Limpiator o el mundo %s NO están activos en Apolo. Abre el mundo y asegúrate de que el robot está presente.', world_apolo);
    end

    try
        % Asegurarse de que la posición inicial esté dentro de los límites del mapa
        if x_inicial_apolo < 0 || x_inicial_apolo > 10 || y_inicial_apolo < 0 || y_inicial_apolo > 33
            error('Posición inicial fuera de los límites del mapa.');
        end
        
        apoloSetRobotPose_S('Limpiator', x_inicial_apolo, y_inicial_apolo, theta_inicial_apolo, world_apolo);
        apoloUpdate(world_apolo);
        pause(0.2);
        pos_actual = apoloGetLocationMRobot('Limpiator', world_apolo);
        disp(['[TELETRANSPORTE] Posición inicial en Apolo: X=', num2str(pos_actual(1)), ', Y=', num2str(pos_actual(2)), ', Theta=', num2str(pos_actual(3))]);
    catch ME
        disp(['[TELETRANSPORTE] Error al establecer la posición inicial en Apolo: ', ME.message]);
    end
    
    try
        % === CONFIGURACIÓN MANUAL DE OBJETIVO Y OFFSET INICIAL ===
        % Modifica aquí el objetivo y los offsets de transformación Apolo->Mapa
        objetivo_apolo_x = 5; % Objetivo X en Apolo (vertical, metros)
        objetivo_apolo_y = 10; % Ajustar a un valor dentro de los límites (horizontal, metros)

        % Asegurarse de que el objetivo esté dentro de los límites del mapa
        if objetivo_apolo_x < 0 || objetivo_apolo_x > 10 || objetivo_apolo_y < 0 || objetivo_apolo_y > 33
            error('Objetivo fuera de los límites del mapa.');
        end

        offset_x = 0; % Offset X para transformar Apolo->Mapa (mantener 0 para sistema Apolo)
        offset_y = 0; % Offset Y para transformar Apolo->Mapa (mantener 0 para sistema Apolo)
        offset_theta = 0; % Offset de orientación (radianes)

        % Guardar en opciones para que Localizacion_S los use
        opciones.offset_x = offset_x;
        opciones.offset_y = offset_y;
        opciones.offset_theta = offset_theta;
        opciones.objetivo_apolo = [objetivo_apolo_x, objetivo_apolo_y];

        % Paso 1: Cargar o crear mapa de ocupación
        [mapa, info_mapa] = crear_mapa_ocupacion_S();
        resolucion = info_mapa.resolucion;
        mapa_filas = size(mapa, 1);
        mapa_cols = size(mapa, 2);
        altura_mapa = 10; % metros
        ancho_mapa = 33;  % metros

        % Paso 2: Calibración inicial
        disp("🔄 Paso 2: Realizando calibración inicial...");
        try
            % Usar calibracion.xml para la calibración
            calibracion_inicial_S(struct("num_iteraciones", 3, "usar_balizas", true, "debug", opciones.debug, "xml_calibracion_path", "EntornoBasico.xml"));
            if exist('calibracion_robot.mat', 'file')
                load('calibracion_robot.mat');
                disp(['ℹ️ Factores de calibración: Distancia=', num2str(calibracion.factor_distancia), ', Ángulo=', num2str(calibracion.factor_angulo)]);
            end
        catch ME
            disp(['⚠️ Error en calibración inicial: ', ME.message, '. Continuando sin calibración.']);
        end
        
        % Paso 3: Localización inicial
        disp("🔄 Paso 3: Obteniendo localización inicial...");
        opciones.archivo_xml = 'root1.xml'; % Para la localización, usar el mapa root1
        opciones.angulo_correccion = 60;    % Offset de orientación en grados
        [x, y, theta] = Localizacion_S(opciones);
        disp(['[DEBUG] Posición inicial antes de conversión: (', num2str(x), ',', num2str(y), ')']);
        [fila_ini, col_ini] = apolo2matriz(x, y);
        disp(['[DEBUG] Conversión a mapa binario: fila=', num2str(fila_ini), ', columna=', num2str(col_ini)]);
        if fila_ini < 1 || fila_ini > mapa_filas || col_ini < 1 || col_ini > mapa_cols
            disp('[ERROR] Posición inicial fuera del mapa. Ajustando al centro.');
            fila_ini = round(mapa_filas/2);
            col_ini = round(mapa_cols/2);
            disp(['[DEBUG] Nueva posición inicial ajustada: fila=', num2str(fila_ini), ', columna=', num2str(col_ini)]);
        end
        disp(['📍 Posición inicial: (', num2str(x), ',', num2str(y), ',', num2str(theta), ')']);
        
        % === OBJETIVO EN CELDA ===
        % Usar directamente las coordenadas de Apolo para el objetivo en celda
        [fila_obj, col_obj] = apolo2matriz(objetivo_apolo_x, objetivo_apolo_y);
        disp(['[DEBUG] Objetivo Apolo: (', num2str(objetivo_apolo_x), ',', num2str(objetivo_apolo_y), ')']);
        disp(['[DEBUG] Conversión objetivo a mapa binario: fila=', num2str(fila_obj), ', columna=', num2str(col_obj)]);
        if fila_obj < 1 || fila_obj > mapa_filas || col_obj < 1 || col_obj > mapa_cols
            disp('[ERROR] Objetivo fuera del mapa. Ajustando al centro.');
            fila_obj = round(mapa_filas/2);
            col_obj = round(mapa_cols/2);
            disp(['[DEBUG] Nueva posición objetivo ajustada: fila=', num2str(fila_obj), ', columna=', num2str(col_obj)]);
        end

        if opciones.visualizar
            figure(100);
            imshow(~mapa, 'InitialMagnification', 'fit');
            colormap(gray);
            hold on;
            scatter(col_obj, fila_obj, 100, 'g', 'filled', 'DisplayName', 'Objetivo Manual');
            scatter(col_ini, fila_ini, 100, 'b', 'filled', 'DisplayName', 'Inicio');
            legend('show');
            title('Mapa de Ocupación con Objetivo e Inicio');
            hold off;
        end
        
        % Paso 4: Planificar exploración
        disp("🔄 Paso 4: Planificando exploración...");
        % mensajero_exploracion_S espera un objeto binaryOccupancyMap o una matriz lógica
        ruta_exploracion = mensajero_exploracion_S(mapa, [fila_ini, col_ini], struct( ...
            'max_puntos', 15, ...
            'visualizar', opciones.visualizar, ...
            'debug', opciones.debug, ...
            'target_pos', [fila_obj, col_obj] ...
        ));
        if isempty(ruta_exploracion)
            disp('❌ No se pudo planificar una ruta de exploración. Abortando navegación.');
            return;
        end
        
        % Paso 5: Verificar obstáculos en la ruta
        disp("🔄 Paso 5: Verificando obstáculos en la ruta...");
        % Pasar info_mapa a obstaculos_S para que pueda usar las transformaciones correctas
        opciones_obstaculos = opciones;
        opciones_obstaculos.info_mapa = info_mapa;
        opciones_obstaculos.objetivo_apolo = [fila_obj, col_obj];
        ruta_final = obstaculos_S(mapa, ruta_exploracion, opciones_obstaculos);
        if isempty(ruta_final)
            disp('❌ No se pudo obtener una ruta final válida. Abortando navegación.');
            return;
        end
        
        % Visualizar ruta final si se solicita
        if opciones.visualizar
            figure(100);
            hold on;
            plot(ruta_exploracion(:,1), ruta_exploracion(:,2), 'r-o', 'LineWidth', 1, 'DisplayName', 'Ruta Exploración');
            plot(ruta_final(:,1), ruta_final(:,2), 'g-*', 'LineWidth', 2, 'DisplayName', 'Ruta Final');
            scatter(col_ini, fila_ini, 100, 'b', 'filled', 'DisplayName', 'Inicio');
            scatter(col_obj, fila_obj, 100, 'g', 'filled', 'DisplayName', 'Objetivo');
            legend('show');
            title('Plan de Navegación');
            hold off;
        end
        
        % Paso 6: Ejecutar movimientos
        disp("🔄 Paso 6: Ejecutando movimientos...");
        
        % Inicializar variables para seguimiento
        posiciones = [x, y, theta];
        errores_odometria = [];
        
        % Limitar a un número razonable de puntos para prueba
        num_puntos = min(size(ruta_final, 1), 5); % Limitar a 5 puntos para una prueba rápida
        
        % Inicialización de sensores ultrasónicos
        ultrasonic_sensors = struct();
        ultrasonic_sensors.frente = struct('name', 'frente', 'position', [0.2, 0, 0.3], 'orientation', [0, -0.4, 0]);
        ultrasonic_sensors.izquierda = struct('name', 'izquierda', 'position', [0.18, 0.2, 0.3], 'orientation', [0, -0.4, 1.5]);
        ultrasonic_sensors.derecha = struct('name', 'derecha', 'position', [0.18, -0.2, 0.3], 'orientation', [0, -0.4, -1.5]);

        % Verificación de compatibilidad de sensores
        if ~isfield(ultrasonic_sensors, 'frente') || ~isfield(ultrasonic_sensors, 'izquierda') || ~isfield(ultrasonic_sensors, 'derecha')
            error('Faltan sensores ultrasónicos necesarios.');
        end

        % Activar sensores
        for i = fieldnames(ultrasonic_sensors)'
            sensor = ultrasonic_sensors.(i{1});
            % Aquí se debe agregar el código para activar el sensor
            disp(['Activando sensor: ', sensor.name]);
        end
        
        % Definición de variables necesarias
        datos_sensores = obtener_datos_sensores(); % Función que obtiene datos de los sensores
        balizas_detectadas = detectar_balizas(); % Función que detecta balizas cercanas
        
        % Aplicar corrección a los datos de los sensores
        % for i = 1:length(datos_sensores)
        %     % Resetear el ángulo a su valor original antes de aplicar la corrección
        %     datos_sensores(i).angulo = mod(datos_sensores(i).angulo, 360);
        %     % Aplicar la corrección solo si el ángulo no ha sido ajustado previamente
        %     if datos_sensores(i).angulo + angulo_correccion >= 0 && datos_sensores(i).angulo + angulo_correccion < 360
        %         datos_sensores(i).angulo = datos_sensores(i).angulo + angulo_correccion;
        %     end
        % end
        
        for i = 1:num_puntos
            fila_dest = ruta_final(i,1);
            col_dest = ruta_final(i,2);
            [x_dest, y_dest] = matriz2apolo(fila_dest, col_dest);
            disp(['[DEBUG] Moviendo a celda (', num2str(fila_dest), ',', num2str(col_dest), ') -> Apolo: (', num2str(x_dest), ',', num2str(y_dest), ')']);
            
            % Mover el robot en Apolo a la posición estimada por el algoritmo
            % Esto simula el teletransporte para mantener la visualización sincronizada
            try
                apoloSetRobotPose_S(opciones.robot_name, x_dest, y_dest, theta, opciones.world);
                apoloUpdate(opciones.world);
                pause(0.1); % Pequeña pausa para la visualización
            catch ME
                if opciones.debug
                    disp(['⚠️ Error al interactuar con Apolo (apoloSetRobotPose_S): ', ME.message]);
                end
            end

            % Relocalización
            [x_actual_apolo, y_actual_apolo, theta_actual_apolo] = Localizacion_S(opciones);
            disp(['[DEBUG] Nueva posición Apolo: (', num2str(x_actual_apolo), ',', num2str(y_actual_apolo), ')']);
            [fila_actual, col_actual] = apolo2matriz(x_actual_apolo, y_actual_apolo);
            disp(['[DEBUG] Conversión nueva posición a mapa binario: fila=', num2str(fila_actual), ', columna=', num2str(col_actual)]);
            if fila_actual < 1 || fila_actual > mapa_filas || col_actual < 1 || col_actual > mapa_cols
                disp('[ERROR] Nueva posición fuera del mapa. Ajustando al centro.');
                fila_actual = round(mapa_filas/2);
                col_actual = round(mapa_cols/2);
            end
            
            % Calcular error de odometría (distancia entre el punto objetivo y la posición real)
            error_dist = sqrt((ruta_final(i,1) - x_actual_apolo)^2 + (ruta_final(i,2) - y_actual_apolo)^2);
            errores_odometria = [errores_odometria; error_dist];
            
            % Registrar posición
            posiciones = [posiciones; x_actual_apolo, y_actual_apolo, theta_actual_apolo];
            
            % Corregir odometría si es necesario
            if error_dist > 0.5
                disp(['⚠️ Error de posición significativo: ', num2str(error_dist), ' metros. Corrigiendo odometría...']);
                % Pasar el mapa y la información del mapa a correccion_balizas_S
                opciones_correccion = opciones;
                opciones_correccion.mapa_actual = mapa;
                opciones_correccion.info_mapa = info_mapa;
                correccion_balizas_S(opciones_correccion); 
            end
            
            % Actualizar mapa si se detectan obstáculos (simplificado)
            try
                datos_sensores = obtener_datos_sensores();
                uc0 = datos_sensores.uc0;
                ul1 = datos_sensores.ul1;
                ur1 = datos_sensores.ur1;
                if any(isnan([uc0, ul1, ur1])) || isempty([uc0, ul1, ur1])
                    disp('⚠️ Lectura de sensores inválida (NaN o vacía). Saltando actualización de obstáculos.');
                    continue;
                end
                if min([uc0, ul1, ur1]) < 0.5
                    disp('⚠️ Obstáculo cercano detectado, actualizando mapa...');
                    angulo_robot_apolo = theta_actual_apolo;
                    dist_min_sensor = min([uc0, ul1, ur1]);
                    x_obstaculo_apolo = x_actual_apolo + dist_min_sensor * cos(angulo_robot_apolo);
                    y_obstaculo_apolo = y_actual_apolo + dist_min_sensor * sin(angulo_robot_apolo);
                    columna = 33 - y_obstaculo_apolo;
                    fila    = 10 - x_obstaculo_apolo;
                    columna = round(columna);
                    fila = round(fila);
                    if columna >= 1 && columna <= mapa_cols && fila >= 1 && fila <= mapa_filas
                        mapa(fila, columna) = false;
                        disp(['✅ Obstáculo marcado en el mapa en (', num2str(x_obstaculo_apolo), ',', num2str(y_obstaculo_apolo), ') (Apolo) -> (', num2str(fila), ',', num2str(columna), ') (Mapa MATLAB)']);
                    else
                        disp('⚠️ Obstáculo detectado fuera de los límites del mapa.');
                    end
                end
            catch ME_sensor_update
                if opciones.debug
                    disp(['⚠️ Error al actualizar mapa por sensores: ', ME_sensor_update.message]);
                end
            end
        end
        
        % Paso 7: Análisis de resultados
        disp("🔄 Paso 7: Analizando resultados...");
        
        % Calcular estadísticas de error
        if ~isempty(errores_odometria)
            error_medio = mean(errores_odometria);
            error_max = max(errores_odometria);
            disp(['📊 Error medio de posición: ', num2str(error_medio), ' metros']);
            disp(['📊 Error máximo de posición: ', num2str(error_max), ' metros']);
        else
            disp('📊 No se registraron errores de odometría.');
        end
        
        % Guardar el mapa de ocupación actualizado
        try
            save('mapa_ocupacion_final.mat', 'mapa');
            disp('Mapa de ocupación guardado como mapa_ocupacion_final.mat');
        catch ME
            disp(['Error al guardar el mapa: ', ME.message]);
        end

        % Informe de resultados
        error_medio = calcular_error_medio(); % Función que calcula el error medio
        error_maximo = calcular_error_maximo(); % Función que calcula el error máximo

        fprintf('📊 Error medio de posición: %.2f metros\n', error_medio);
        fprintf('📊 Error máximo de posición: %.2f metros\n', error_maximo);

        disp("=== Navegación Completada ===");
        
    catch ME_main
        disp(['❌ Error crítico en navegador_principal: ', ME_main.message]);
        if opciones.debug
            rethrow(ME_main);
        end
    end

    % Finalizar ejecución de manera ordenada
    try
        % Aquí se pueden liberar recursos o cerrar conexiones si es necesario
        disp('✅ Ejecución finalizada correctamente.');
    catch ME
        disp(['Error al finalizar la ejecución: ', ME.message]);
    end
end

% Función auxiliar para normalizar ángulos a [-pi, pi]
function angulo_normalizado = normalizar_angulo(angulo)
    angulo_normalizado = mod(angulo + pi, 2*pi) - pi;
end

% Función para obtener datos de los sensores (simulación segura)
function datos = obtener_datos_sensores()
    datos = struct();
    try
        datos.uc0 = apoloGetUltrasonicSensor('uc0', 'world1');
    catch
        datos.uc0 = 1.0;
    end
    try
        datos.ul1 = apoloGetUltrasonicSensor('ul1', 'world1');
    catch
        datos.ul1 = 1.0;
    end
    try
        datos.ur1 = apoloGetUltrasonicSensor('ur1', 'world1');
    catch
        datos.ur1 = 1.0;
    end
    try
        datos.laser = apoloGetLaserData('LMS100', 'world1');
    catch
        datos.laser = [];
    end
    if any(isnan(struct2array(datos)))
        datos.uc0 = 1.0;
        datos.ul1 = 1.0;
        datos.ur1 = 1.0;
        datos.laser = [];
    end
end

% Función para detectar balizas
function balizas = detectar_balizas()
    % Aquí se debe implementar la lógica para detectar balizas
    balizas = []; % Reemplazar con la lógica real
end

% === FUNCIÓN DE TRANSFORMACIÓN APOLO -> MATRIZ BINARIA CORREGIDA ===
% Ahora devuelve [fila, columna] en ese orden
function [fila, columna] = apolo2matriz(x_apolo, y_apolo)
    fila    = 10 - x_apolo;
    columna = 33 - y_apolo;
    fila = round(fila);
    columna = round(columna);
end

% === FUNCIÓN DE TRANSFORMACIÓN MATRIZ -> APOLO ===
function [x_apolo, y_apolo] = matriz2apolo(fila, columna)
    x_apolo = 10 - fila;  % Conversión de fila a coordenada Apolo
    y_apolo = 33 - columna; % Conversión de columna a coordenada Apolo
end


