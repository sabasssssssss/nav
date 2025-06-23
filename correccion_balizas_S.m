% ... (código anterior sin cambios relevantes)

% Función para normalizar ángulos al rango [-pi, pi]
function angulo = normalizar_angulo(angulo)
    while angulo > pi
        angulo = angulo - 2*pi;
    end
    while angulo < -pi
        angulo = angulo + 2*pi;
    end
end

% La conversión Apolo -> matriz se realiza usando la función global apolo2matriz.