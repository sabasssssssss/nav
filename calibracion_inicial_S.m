function calibracion_inicial_S(opciones)
    % Función para la calibración inicial del robot utilizando datos de un archivo XML.
    % Realiza un procedimiento para determinar factores de escala para la odometría y sesgos de sensores.
    %
    % Parámetros:
    %   opciones - Estructura con parámetros configurables:
    %     - opciones.num_iteraciones: Número de iteraciones de calibración (por defecto: 1)
    %     - opciones.usar_balizas: Indica si se usan balizas para calibración (por defecto: false)
    %     - opciones.xml_calibracion_path: Ruta al archivo XML de calibración (por defecto: 'calibracion.xml')
    %     - opciones.debug: Mostrar mensajes de depuración (por defecto: false)
    
    num_iteraciones = getfield_with_default(opciones, 'num_iteraciones', 1);
    usar_balizas = getfield_with_default(opciones, 'usar_balizas', false);
    debug = getfield_with_default(opciones, 'debug', false);
    xml_calibracion_path = getfield_with_default(opciones, 'xml_calibracion_path', 'calibracion.xml');

    if debug
        disp('ℹ️ Iniciando calibración inicial...');
    end

    % Cargar datos de calibración desde el XML
    balizas_calibracion = [];
    if usar_balizas && exist(xml_calibracion_path, 'file')
        try
            [~, balizas_calibracion] = parse_xml_map_S(xml_calibracion_path);
            if debug
                disp(['✅ Balizas de calibración cargadas desde ', xml_calibracion_path, ': ', num2str(size(balizas_calibracion, 1)), ' balizas encontradas']);
            end
        catch ME
            if debug
                disp(['⚠️ Error al cargar balizas de calibración desde XML (', ME.message, ').']);
            end
        end
    else
        if debug
            disp('⚠️ No se usarán balizas para la calibración o el archivo XML no existe.');
        end
    end

    % Simulación de un proceso de calibración
    factor_distancia = 1.0; % Factor de escala para la distancia
    factor_angulo = 1.0;    % Factor de escala para el ángulo

    for i = 1:num_iteraciones
        if debug
            disp(['   Calibración iteración ', num2str(i), ' de ', num2str(num_iteraciones)]);
        end
        % Aquí iría la lógica real de calibración, por ejemplo:
        % 1. Mover el robot una distancia conocida y medir la odometría.
        % 2. Girar el robot un ángulo conocido y medir la odometría.
        % 3. Usar balizas_calibracion para refinar la posición si usar_balizas es true.
        
        % Simulación de ajuste de factores
        factor_distancia = factor_distancia * (1 + (rand() - 0.5) * 0.01); % Pequeño ajuste aleatorio
        factor_angulo = factor_angulo * (1 + (rand() - 0.5) * 0.01);     % Pequeño ajuste aleatorio

        % Si se usan balizas, simular una corrección basada en ellas
        if usar_balizas && ~isempty(balizas_calibracion)
            % Simular que el robot se mueve cerca de una baliza y corrige su odometría
            % Esto es un placeholder, la lógica real sería más compleja
            factor_distancia = factor_distancia * 0.99; % Simular una mejora en la precisión
            factor_angulo = factor_angulo * 0.99;       % Simular una mejora en la precisión
            if debug
                disp('   Simulando corrección de calibración usando balizas.');
            end
        end
    end

    % Guardar los factores de calibración para que puedan ser usados por Localizacion_S
    calibracion = struct();
    calibracion.factor_distancia = factor_distancia;
    calibracion.factor_angulo = factor_angulo;
    save('calibracion_robot.mat', 'calibracion');

    if debug
        disp('✅ Calibración inicial completada. Factores guardados en calibracion_robot.mat');
        disp(['   Factor Distancia: ', num2str(factor_distancia)]);
        disp(['   Factor Ángulo: ', num2str(factor_angulo)]);
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



