function ruta_filtrada = obstaculos_S(mapa, ruta_inicial, opciones)
    % Función para verificar y ajustar una ruta en presencia de obstáculos.
    % Si se detecta un obstáculo en la ruta, intenta replanificar la sección afectada
    % utilizando el algoritmo A*.
    %
    % Parámetros:
    %   mapa: Matriz lógica que representa el mapa de ocupación (true = libre, false = ocupado).
    %   ruta_inicial: Matriz de Nx2 con los puntos [x, y] de la ruta inicial (coordenadas de mapa MATLAB).
    %   opciones - Estructura con parámetros configurables:
    %     - opciones.actualizar_mapa: Indica si se actualiza el mapa con nuevos obstáculos (por defecto: false).
    %     - opciones.visualizar: Mostrar la ruta en el mapa (por defecto: true).
    %     - opciones.debug: Mostrar mensajes de depuración (por defecto: false).
    %     - opciones.info_mapa: Estructura con la información del mapa (resolucion, min_apolo_x, etc.)
    
    actualizar_mapa = getfield_with_default(opciones, 'actualizar_mapa', false);
    visualizar = getfield_with_default(opciones, 'visualizar', true);
    debug = getfield_with_default(opciones, 'debug', false);
    info_mapa = getfield_with_default(opciones, 'info_mapa', []); % Obtener info_mapa de opciones

    if debug
        disp('ℹ️ Verificando obstáculos en la ruta y replanificando si es necesario...');
    end

    ruta_filtrada = ruta_inicial; % Inicialmente, la ruta filtrada es la ruta inicial
    [filas_mapa, cols_mapa] = size(mapa);

    % Parámetros del mapa
    if exist('info_mapa','var')
        resolucion = info_mapa.resolucion;
        offset_x = info_mapa.offset_apolo_x_a_matlab_y;
        offset_y = info_mapa.offset_apolo_y_a_matlab_x;
    else
        resolucion = 1; offset_x = 0; offset_y = 0;
    end

    % Iterar sobre la ruta para verificar obstáculos
    i = 1;
    while i <= size(ruta_filtrada, 1)
        x_punto = round(ruta_filtrada(i, 1)); % Coordenadas de celda MATLAB
        y_punto = round(ruta_filtrada(i, 2)); % Coordenadas de celda MATLAB
        
        % Asegurarse de que las coordenadas estén dentro de los límites del mapa
        if x_punto < 1 || x_punto > 33 || y_punto < 1 || y_punto > 10
            if debug
                disp(['⚠️ Coordenadas fuera de límites: (', num2str(x_punto), ',', num2str(y_punto), ')']);
            end
            ruta_filtrada = ruta_filtrada(1:i-1, :);
            if debug
                disp('✂️ Ruta cortada debido a coordenadas fuera de límites. Se requiere replanificación.');
            end
            return;
        end
        
        % El mapa ya está en el sistema de coordenadas de MATLAB (fila, columna)
        if ~mapa(y_punto, x_punto) % Si el punto está ocupado (false = ocupado)
            if debug
                disp(['⚠️ Obstáculo detectado en el punto de ruta (fila, columna): (', num2str(y_punto), ',', num2str(x_punto), ')']);
            end
            
            % Punto de inicio para la replanificación: el punto anterior si existe, o el punto actual si es el primero
            if i == 1
                start_replan = ruta_filtrada(i, :); % Intentar replanificar desde el punto actual
            else
                start_replan = ruta_filtrada(i-1, :);
            end
            
            % El punto objetivo para la replanificación es el último punto de la ruta original
            end_replan = ruta_inicial(end, :);
            
            if debug
                disp(['🔄 Replanificando desde (celdas): (', num2str(start_replan(2)), ',', num2str(start_replan(1)), ')']);
            end
            
            % Llamar a mensajero_exploracion_S para replanificar
            opciones_replan = struct('visualizar', false, 'debug', debug, 'target_pos', end_replan);
            nueva_ruta_segmento = mensajero_exploracion_S(mapa, start_replan, opciones_replan);
            
            if ~isempty(nueva_ruta_segmento)
                if debug
                    disp('✅ Ruta replanificada con éxito.');
                end
                % Concatenar la parte ya recorrida de la ruta con la nueva ruta
                ruta_filtrada = [ruta_filtrada(1:i-1, :); nueva_ruta_segmento];
                i = size(ruta_filtrada, 1); % Saltar al final para terminar la verificación en esta iteración
            else
                if debug
                    disp('❌ No se pudo replanificar la ruta. El robot podría estar atascado.');
                end
                ruta_filtrada = ruta_filtrada(1:i-1, :); % Cortar la ruta hasta el punto anterior al obstáculo
                return; % Salir de la función
            end
        end
        i = i + 1;
    end

    if debug
        disp('✅ Verificación de obstáculos completada. Ruta validada.');
    end

    % Si se requiere actualizar el mapa con nuevos obstáculos (por ejemplo, detectados por sensores)
    if actualizar_mapa
        % Esta lógica se maneja en navegador_principal_S, donde se actualiza el mapa
        % con las lecturas de los sensores ultrasónicos.
        if debug
            disp('ℹ️ La actualización del mapa con nuevos obstáculos se gestiona en navegador_principal_S.');
        end
    end

    % Permitir definir un objetivo manual en el sistema Apolo/XML para pruebas
    if isfield(opciones, 'objetivo_apolo') && ~isempty(opciones.objetivo_apolo)
        fila = opciones.objetivo_apolo(1);
        columna = opciones.objetivo_apolo(2);
        if debug
            disp(['[DEBUG] Tamaño del mapa: filas=', num2str(filas_mapa), ', columnas=', num2str(cols_mapa)]);
            disp(['[DEBUG] Objetivo manual recibido: fila=', num2str(fila), ', columna=', num2str(columna)]);
            if fila >= 1 && fila <= filas_mapa && columna >= 1 && columna <= cols_mapa
                disp(['[DEBUG] Valor en mapa(fila, columna): ', num2str(mapa(fila, columna))]);
            end
        end
        if fila < 1 || fila > filas_mapa || columna < 1 || columna > cols_mapa || ~mapa(fila, columna)
            disp('❌ El objetivo manual está fuera de los límites o en un obstáculo.');
            ruta_filtrada = [];
            return;
        end
        opciones_replan = struct('visualizar', true, 'debug', debug, 'target_pos', [fila, columna]);
        nueva_ruta = mensajero_exploracion_S(mapa, ruta_inicial(1,:), opciones_replan);
        if isempty(nueva_ruta)
            disp('❌ No se pudo planificar una ruta hacia el objetivo manual.');
            ruta_filtrada = [];
            return;
        end
        ruta_filtrada = nueva_ruta;
        return;
    end

    % Visualizar la ruta si se solicita
    if visualizar && debug % Solo visualizar si debug está activo
        figure;
        imshow(~mapa, 'InitialMagnification', 'fit'); % Invertir para que los obstáculos sean negros
        colormap(gray);
        hold on;
        plot(ruta_inicial(:,1), ruta_inicial(:,2), 'r-o', 'LineWidth', 1, 'DisplayName', 'Ruta Inicial');
        plot(ruta_filtrada(:,1), ruta_filtrada(:,2), 'g-o', 'LineWidth', 2, 'DisplayName', 'Ruta Filtrada (Obstáculos)');
        legend('show');
        title('Ruta con Verificación de Obstáculos');
        hold off;
    end

    % Al final de la función, después de la verificación de obstáculos y antes de devolver ruta_filtrada:
    if size(ruta_filtrada,1) < 2
        disp('❌ Ruta filtrada demasiado corta. Abortando.');
        ruta_filtrada = [];
        return;
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

% La conversión de coordenadas se delega ahora en la función global
% apolo2matriz.m para mantener coherencia en todo el código.


