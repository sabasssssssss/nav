function [estado_corregido, matriz_covarianza_actualizada] = kalman_filter_update_S(estado_predicho, matriz_covarianza_pred, balizas, umbral_distancia, var_dist, var_ang, opciones)
    % Función modularizada para la actualización del filtro de Kalman.
    % Realiza la fase de corrección del EKF utilizando mediciones de balizas.
    %
    % Parámetros:
    %   estado_predicho: Vector de estado predicho [x; y; theta].
    %   matriz_covarianza_pred: Matriz de covarianza predicha.
    %   balizas: Matriz de balizas conocidas [x_baliza, y_baliza].
    %   umbral_distancia: Distancia máxima para considerar una baliza detectada.
    %   var_dist: Varianza de la medición de distancia de la baliza.
    %   var_ang: Varianza de la medición de ángulo de la baliza.
    %   opciones: Estructura con opciones adicionales (e.g., debug, datos_sensores).
    
    debug = getfield_with_default(opciones, 'debug', false);
    datos_sensores = getfield_with_default(opciones, 'datos_sensores', struct());

    estado_corregido = estado_predicho; % Inicialmente, el estado corregido es el predicho
    matriz_covarianza_actualizada = matriz_covarianza_pred; % Inicialmente, la covarianza corregida es la predicha

    % Extraer componentes del estado predicho
    x_pred = estado_predicho(1);
    y_pred = estado_predicho(2);
    theta_pred = estado_predicho(3);

    % Buscar balizas cercanas para corrección
    balizas_detectadas = [];
    for i = 1:size(balizas, 1)
        bx = balizas(i,1);
        by = balizas(i,2);
        
        % Calcular distancia euclidiana a la baliza
        dist_euclidiana = sqrt((bx - x_pred)^2 + (by - y_pred)^2);
        
        if dist_euclidiana < umbral_distancia
            % Simular medición de distancia y ángulo a la baliza
            % En un sistema real, estas serían las lecturas de los sensores
            z_dist = dist_euclidiana + (rand() - 0.5) * 0.1; % Añadir ruido simulado
            z_ang = atan2(by - y_pred, bx - x_pred) - theta_pred + (rand() - 0.5) * 0.05; % Añadir ruido simulado
            z_ang = normalizar_angulo(z_ang);
            
            balizas_detectadas = [balizas_detectadas; bx, by, z_dist, z_ang];
        end
    end

    if ~isempty(balizas_detectadas)
        if debug
            disp([num2str(size(balizas_detectadas, 1)), ' baliza(s) detectada(s) para corrección.']);
        end

        % Construir vector de mediciones (z) y matriz Jacobiana (H)
        z = [];
        H = [];
        R = []; % Matriz de covarianza de las mediciones

        for k = 1:size(balizas_detectadas, 1)
            bx = balizas_detectadas(k, 1);
            by = balizas_detectadas(k, 2);
            z_dist_medida = balizas_detectadas(k, 3);
            z_ang_medida = balizas_detectadas(k, 4);

            % Medición esperada (h(x)) basada en el estado predicho
            dist_esperada = sqrt((bx - x_pred)^2 + (by - y_pred)^2);
            ang_esperado = normalizar_angulo(atan2(by - y_pred, bx - x_pred) - theta_pred);

            % Añadir a z (innovación)
            z = [z; z_dist_medida - dist_esperada; z_ang_medida - ang_esperado];

            % Calcular Jacobiano H para esta baliza
            dist_sq = (bx - x_pred)^2 + (by - y_pred)^2;
            dist = sqrt(dist_sq);

            h11 = -(bx - x_pred) / dist;
            h12 = -(by - y_pred) / dist;
            h13 = 0;

            h21 = (by - y_pred) / dist_sq;
            h22 = -(bx - x_pred) / dist_sq;
            h23 = -1;

            H_k = [
                h11, h12, h13;
                h21, h22, h23
            ];
            H = [H; H_k];

            % Añadir a R (matriz de covarianza de las mediciones)
            R_k = diag([var_dist, var_ang]);
            R = blkdiag(R, R_k);
        end

        % Calcular la ganancia de Kalman (K)
        S = H * matriz_covarianza_pred * H' + R;
        K = matriz_covarianza_pred * H' * inv(S);

        % Actualizar el estado y la matriz de covarianza
        estado_corregido = estado_predicho + K * z;
        matriz_covarianza_actualizada = (eye(size(K, 1)) - K * H) * matriz_covarianza_pred;
        
        % Normalizar el ángulo en el estado corregido
        estado_corregido(3) = normalizar_angulo(estado_corregido(3));

        if debug
            disp('✅ Estado corregido por filtro de Kalman.');
        end
    else
        if debug
            disp('ℹ️ No se detectaron balizas cercanas. No se aplicó corrección de Kalman.');
        end
    end

    % Eliminar cualquier fórmula alternativa de transformación. Usar solo apolo2matriz y matriz2apolo.
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


