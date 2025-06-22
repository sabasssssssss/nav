function ruta = mensajero_exploracion_S(mapa, posicion_inicial, opciones)
    % Función para planificar una ruta de exploración utilizando el algoritmo A*.
    % Encuentra una ruta desde una posición inicial hasta un punto objetivo
    % (aleatorio si no se especifica, o un punto dado), evitando obstáculos.
    %
    % Parámetros:
    %   mapa: Matriz lógica que representa el mapa de ocupación (true = libre, false = ocupado).
    %   posicion_inicial: [x, y] del robot en el mapa (coordenadas de celda).
    %   opciones - Estructura con parámetros configurables:
    %     - opciones.max_puntos: Número máximo de puntos en la ruta (no directamente usado por A*, pero puede limitar la longitud de la ruta).
    %     - opciones.visualizar: Mostrar la ruta en el mapa (por defecto: true).
    %     - opciones.debug: Mostrar mensajes de depuración (por defecto: false).
    %     - opciones.target_pos: [x, y] del punto objetivo específico (opcional). Si no se proporciona, se elige uno aleatorio.
    
    visualizar = getfield_with_default(opciones, 'visualizar', true);
    debug = getfield_with_default(opciones, 'debug', false);
    target_pos = getfield_with_default(opciones, 'target_pos', []); % Nuevo parámetro para el objetivo

    if debug
        disp('ℹ️ Planificando ruta de exploración usando A*...');
    end

    [filas_mapa, cols_mapa] = size(mapa);

    % Convertir posicion_inicial a índices de fila/columna para el mapa
    start_row = round(posicion_inicial(1));
    start_col = round(posicion_inicial(2));
    start_node = [start_row, start_col];

    % Asegurarse de que el punto inicial sea transitable y dentro de los límites
    if start_row < 1 || start_row > filas_mapa || start_col < 1 || start_col > cols_mapa || ~mapa(start_row, start_col)
        if debug
            disp('❌ El punto inicial está fuera de los límites o en un obstáculo. No se puede planificar la ruta.');
            disp(['[DEBUG] Límites: filas 1-', num2str(filas_mapa), ', columnas 1-', num2str(cols_mapa), ']']);
        end
        ruta = [];
        return;
    end

    % Asegurarse de que la posición inicial esté dentro de los límites del mapa
    if posicion_inicial(1) < 0 || posicion_inicial(1) > 10 || posicion_inicial(2) < 0 || posicion_inicial(2) > 33
        error('Posición inicial fuera de los límites del mapa.');
    end
    
    % Asegurarse de que el objetivo esté dentro de los límites del mapa
    if ~isempty(target_pos) && (target_pos(1) < 0 || target_pos(1) > 10 || target_pos(2) < 0 || target_pos(2) > 33)
        error('Objetivo fuera de los límites del mapa.');
    end

    if debug
        disp(['[DEBUG] Posición inicial: (', num2str(start_row), ',', num2str(start_col), ')']);
        disp(['[DEBUG] target_pos antes de conversión: ', mat2str(target_pos)]);
    end
    % Determinar el punto objetivo
    end_node = [];
    if length(target_pos) == 2 && all(target_pos == round(target_pos))
        disp(['[DEBUG] target_pos: ', mat2str(target_pos)]);
        end_row = target_pos(1);
        end_col = target_pos(2);
    else
        disp(['[DEBUG] Convierte target_pos desde Apolo: ', mat2str(target_pos)]);
        [end_row, end_col] = apolo2matriz(target_pos(1), target_pos(2));
    end
    if debug
        disp(['[DEBUG] Tamaño del mapa: filas=', num2str(filas_mapa), ', columnas=', num2str(cols_mapa)]);
        disp(['[DEBUG] Objetivo convertido: fila=', num2str(end_row), ', columna=', num2str(end_col)]);
        if end_row >= 1 && end_row <= filas_mapa && end_col >= 1 && end_col <= cols_mapa
            disp(['[DEBUG] Valor en mapa(fila, columna): ', num2str(mapa(end_row, end_col))]);
        end
    end
    if end_row < 1 || end_row > filas_mapa || end_col < 1 || end_col > cols_mapa %|| ~mapa(end_row, end_col)
        if debug
            disp('❌ El punto objetivo especificado está fuera de los límites. No se puede planificar la ruta.');
        end
        ruta = [];
        return;
    end
    end_node = [end_row, end_col];
    
    if debug
        disp(['Inicio (celda): (', num2str(start_node(2)), ',', num2str(start_node(1)), ')']);
        disp(['Objetivo (celda): (', num2str(end_node(2)), ',', num2str(end_node(1)), ')']);
    end

    % Implementación del algoritmo A*
    % g_score: Costo del camino desde el inicio hasta el nodo actual
    % f_score: Costo total estimado desde el inicio hasta el objetivo a través del nodo actual (g_score + h_score)
    % came_from: Para reconstruir el camino

    g_score = inf(filas_mapa, cols_mapa);
    g_score(start_node(1), start_node(2)) = 0;

    f_score = inf(filas_mapa, cols_mapa);
    f_score(start_node(1), start_node(2)) = heuristic(start_node, end_node);

    open_set = containers.Map('KeyType', 'char', 'ValueType', 'double'); % Almacena f_score
    open_set(node_to_key(start_node)) = f_score(start_node(1), start_node(2));

    came_from = containers.Map('KeyType', 'char', 'ValueType', 'char'); % Almacena la clave del nodo padre

    while open_set.Count > 0
        % Obtener el nodo con el f_score más bajo del open_set
        current_key = '';
        min_f_score = inf;
        keys = open_set.keys;
        for i = 1:length(keys)
            key = keys{i};
            if open_set(key) < min_f_score
                min_f_score = open_set(key);
                current_key = key;
            end
        end
        current_node = key_to_node(current_key);

        if isequal(current_node, end_node)
            ruta = reconstruct_path(came_from, current_node, filas_mapa); % Reconstruir y convertir a coordenadas de mapa
            if debug
                disp(['✅ Ruta encontrada con ', num2str(size(ruta, 1)), ' puntos.']);
            end
            if visualizar && isempty(target_pos) % Solo visualizar si no es una replanificación interna
                figure;
                imshow(~mapa, 'InitialMagnification', 'fit');
                colormap(gray);
                hold on;
                plot(ruta(:,1), ruta(:,2), 'r-o', 'LineWidth', 2, 'DisplayName', 'Ruta A*');
                scatter(posicion_inicial(1), posicion_inicial(2), 100, 'b', 'filled', 'DisplayName', 'Inicio');
                scatter(target_pos(1), target_pos(2), 100, 'g', 'filled', 'DisplayName', 'Objetivo');
                legend('show');
                title('Ruta de Exploración A*');
                hold off;
            end
            if size(ruta,1) < 2
                disp('❌ Ruta demasiado corta. Abortando.');
                ruta = [];
                return;
            end
            return;
        end

        remove(open_set, current_key);

        % Vecinos (8 direcciones: arriba, abajo, izquierda, derecha, y diagonales)
        neighbors_offsets = [
            -1, 0;   % Arriba
            1, 0;    % Abajo
            0, -1;   % Izquierda
            0, 1;    % Derecha
            -1, -1;  % Arriba-Izquierda
            -1, 1;   % Arriba-Derecha
            1, -1;   % Abajo-Izquierda
            1, 1    % Abajo-Derecha
        ];

        for i = 1:size(neighbors_offsets, 1)
            neighbor = current_node + neighbors_offsets(i,:);
            
            % Verificar límites del mapa
            if neighbor(1) >= 1 && neighbor(1) <= filas_mapa && ...
               neighbor(2) >= 1 && neighbor(2) <= cols_mapa && ...
               mapa(neighbor(1), neighbor(2)) % Asegurarse de que no sea un obstáculo

                % Calcular costo tentativo del camino al vecino
                tentative_g_score = g_score(current_node(1), current_node(2)) + distance_cost(current_node, neighbor);

                if tentative_g_score < g_score(neighbor(1), neighbor(2))
                    came_from(node_to_key(neighbor)) = node_to_key(current_node);
                    g_score(neighbor(1), neighbor(2)) = tentative_g_score;
                    f_score(neighbor(1), neighbor(2)) = tentative_g_score + heuristic(neighbor, end_node);
                    if ~isKey(open_set, node_to_key(neighbor))
                        open_set(node_to_key(neighbor)) = f_score(neighbor(1), neighbor(2));
                    end
                end
            end
        end
    end

    if debug
        disp('❌ No se encontró una ruta.');
    end
    ruta = []; % No se encontró ruta
end

% Función heurística (distancia euclidiana)
function d = heuristic(node1, node2)
    d = sqrt((node1(1) - node2(1))^2 + (node1(2) - node2(2))^2);
end

% Costo de distancia entre nodos (1 para adyacentes, sqrt(2) para diagonales)
function cost = distance_cost(node1, node2)
    if abs(node1(1) - node2(1)) == 1 && abs(node1(2) - node2(2)) == 1
        cost = sqrt(2); % Diagonal
    else
        cost = 1; % Horizontal o vertical
    end
end

% Reconstruir el camino desde el nodo final hasta el inicio
function path = reconstruct_path(came_from, current_node, filas_mapa)
    total_path = {};
    max_iter = 10000; % Protección contra bucles infinitos
    iter = 0;
    while isKey(came_from, node_to_key(current_node)) && iter < max_iter
        total_path{end+1} = current_node; %#ok<AGROW>
        current_node = key_to_node(came_from(node_to_key(current_node)));
        iter = iter + 1;
    end
    total_path{end+1} = current_node; %#ok<AGROW>
    total_path = fliplr(total_path); % Invertir para ir del inicio al fin

    if isempty(total_path)
        disp('❌ Error: La ruta reconstruida está vacía.');
        path = [];
        return;
    end

    % Convertir a matriz de Nx2 y a coordenadas de mapa (fila, columna)
    path = zeros(length(total_path), 2);
    for i = 1:length(total_path)
        node = total_path{i};
        if any(node < 1) || node(1) > filas_mapa
            disp(['❌ Nodo fuera de rango en la ruta: ', mat2str(node)]);
            path = path(1:i-1, :); % Truncar la ruta hasta el último válido
            return;
        end
        path(i, 1) = node(1); % Fila
        path(i, 2) = node(2); % Columna
    end
end

% Convertir nodo [fila, columna] a clave de string para el mapa
function key = node_to_key(node)
    key = [num2str(node(1)), '_', num2str(node(2))];
end

% Convertir clave de string a nodo [fila, columna]
function node = key_to_node(key)
    parts = strsplit(key, '_');
    node = [str2double(parts{1}), str2double(parts{2})];
end

% Función auxiliar para obtener campos de estructura con valores por defecto
function valor = getfield_with_default(estructura, campo, valor_default)
    if isfield(estructura, campo)
        valor = estructura.(campo);
    else
        valor = valor_default;
    end
end

% La conversión de coordenadas se realiza mediante la función global
% apolo2matriz.m para evitar duplicaciones.

