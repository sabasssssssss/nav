function apoloSetRobotPose_S(robot_name, x_mapa_deseado, y_mapa_deseado, theta_deseado, world)
    % Función para intentar "teletransportar" el robot en Apolo a una posición deseada.
    % Dado que Apolo no tiene una función directa para establecer la pose, esta función
    % calcula el movimiento necesario y lo ejecuta rápidamente.
    %
    % robot_name: Nombre del robot en Apolo (e.g., 'Limpiator')
    % x_mapa_deseado: Coordenada X deseada en el sistema de coordenadas del mapa (metros)
    % y_mapa_deseado: Coordenada Y deseada en el sistema de coordenadas del mapa (metros)
    % theta_deseado: Orientación deseada en el sistema de coordenadas del mapa (radianes)

    % Convertir coordenadas del mapa binario a Apolo usando la convención global
    x_apolo_deseado = 10 - x_mapa_deseado;
    y_apolo_deseado = 33 - y_mapa_deseado;
    theta_apolo_deseado = theta_deseado;

    % Obtener la posición actual del robot en Apolo
    try
        pos_actual_apolo = apoloGetLocationMRobot(robot_name, world); % Requiere Apolo
        x_actual_apolo = pos_actual_apolo(1);
        y_actual_apolo = pos_actual_apolo(2);
        theta_actual_apolo = pos_actual_apolo(3);
    catch ME
        disp([char(9888), ' Error al obtener posición de Apolo (apoloGetLocationMRobot): ', ME.message]);
        % Valores por defecto para simulación si Apolo no está disponible
        x_actual_apolo = 0;
        y_actual_apolo = 0;
        theta_actual_apolo = 0;
    end

    % Calcular la diferencia de posición y orientación
    delta_x_apolo = x_apolo_deseado - x_actual_apolo;
    delta_y_apolo = y_apolo_deseado - y_actual_apolo;
    delta_theta_apolo = theta_apolo_deseado - theta_actual_apolo;
    delta_theta_apolo = normalizar_angulo(delta_theta_apolo); % Normalizar el ángulo

    % Calcular la distancia lineal a mover
    distancia_lineal = sqrt(delta_x_apolo^2 + delta_y_apolo^2);

    % Calcular el ángulo de giro para apuntar al destino
    angulo_hacia_destino = atan2(delta_y_apolo, delta_x_apolo);
    giro_inicial = angulo_hacia_destino - theta_actual_apolo;
    giro_inicial = normalizar_angulo(giro_inicial);

    % Definir una velocidad de movimiento rápida para simular un "teletransporte"
    velocidad_teletransporte_lineal = 10; % m/s
    velocidad_teletransporte_angular = 10; % rad/s

    % Ejecutar el giro inicial
    if abs(giro_inicial) > 0.01 % Solo girar si el ángulo es significativo
        try
            apoloMoveMRobot(robot_name, [0, sign(giro_inicial) * velocidad_teletransporte_angular], abs(giro_inicial) / velocidad_teletransporte_angular, world); % Requiere Apolo
            apoloUpdate(world); % Requiere Apolo
            pause(0.01); % Pequeña pausa para asegurar que el giro se complete
        catch ME
            disp([char(9888), ' Error al interactuar con Apolo (apoloMoveMRobot - giro inicial): ', ME.message]);
        end
    end

    % Ejecutar el movimiento lineal
    if distancia_lineal > 0.01 % Solo avanzar si la distancia es significativa
        try
            apoloMoveMRobot(robot_name, [velocidad_teletransporte_lineal, 0], distancia_lineal / velocidad_teletransporte_lineal, world); % Requiere Apolo
            apoloUpdate(world); % Requiere Apolo
            pause(0.01); % Pequeña pausa para asegurar que el avance se complete
        catch ME
            disp([char(9888), ' Error al interactuar con Apolo (apoloMoveMRobot - avance): ', ME.message]);
        end
    end

    % Ejecutar el giro final para la orientación deseada
    % Obtener la orientación actual después del movimiento lineal
    try
        pos_despues_avance = apoloGetLocationMRobot(robot_name, world); % Requiere Apolo
        theta_despues_avance = pos_despues_avance(3);
    catch ME
        disp([char(9888), ' Error al obtener posición de Apolo (apoloGetLocationMRobot - después avance): ', ME.message]);
        theta_despues_avance = theta_actual_apolo; % Usar el valor inicial si hay error
    end

    giro_final = theta_deseado - theta_despues_avance;
    giro_final = normalizar_angulo(giro_final);

    if abs(giro_final) > 0.01 % Solo girar si el ángulo es significativo
        try
            apoloMoveMRobot(robot_name, [0, sign(giro_final) * velocidad_teletransporte_angular], abs(giro_final) / velocidad_teletransporte_angular, world); % Requiere Apolo
            apoloUpdate(world); % Requiere Apolo
            pause(0.01); % Pequeña pausa
        catch ME
            disp([char(9888), ' Error al interactuar con Apolo (apoloMoveMRobot - giro final): ', ME.message]);
        end
    end

    disp([char(9989), ' Robot ', robot_name, ' teletransportado a (', num2str(x_mapa_deseado), ', ', num2str(y_mapa_deseado), ', ', num2str(theta_deseado), ')']);
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


