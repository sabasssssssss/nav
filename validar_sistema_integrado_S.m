function validar_sistema_integrado_S(opciones)
    % Función para validar el funcionamiento del sistema integrado
    % Ejecuta pruebas con el navegador principal y el control reactivo
    % Analiza los resultados y genera informes de validación
    %
    % Parámetros:
    %   opciones - Estructura con parámetros configurables:
    %     - opciones.ejecutar_navegador: Ejecutar prueba del navegador principal (por defecto: true)
    %     - opciones.ejecutar_reactivo: Ejecutar prueba del control reactivo (por defecto: true)
    %     - opciones.visualizar: Mostrar visualizaciones durante las pruebas (por defecto: false)
    %     - opciones.analizar_resultados: Analizar y mostrar resultados (por defecto: true)
    %     - opciones.guardar_informe: Guardar informe de validación (por defecto: true)
    
    % Configuración de opciones por defecto
    if nargin < 1
        opciones = struct();
    end
    
    % Parámetros configurables con valores por defecto
    ejecutar_navegador = getfield_with_default(opciones, 'ejecutar_navegador', true);
    ejecutar_reactivo = getfield_with_default(opciones, 'ejecutar_reactivo', false);
    visualizar_pruebas = getfield_with_default(opciones, 'visualizar', false);
    analizar_resultados = getfield_with_default(opciones, 'analizar_resultados', true);
    guardar_informe = getfield_with_default(opciones, 'guardar_informe', true);
    
    disp('=== Iniciando Validación del Sistema Integrado ===');
    
    % Inicializar estructura para el informe
    informe = struct();
    informe.fecha = datestr(now);
    informe.pruebas_realizadas = {};
    informe.resultados_navegador = [];
    informe.resultados_reactivo = [];
    informe.analisis = struct();
    
    try
        % Prueba 1: Navegador Principal con Filtro de Kalman
        if ejecutar_navegador
            disp('🔄 Prueba 1: Ejecutando Navegador Principal con Filtro de Kalman...');
            informe.pruebas_realizadas{end+1} = 'Navegador Principal con Kalman';
            
            % Configurar opciones para la prueba
            opciones_navegador = struct();
            opciones_navegador.visualizar = visualizar_pruebas;
            opciones_navegador.guardar_historico = true; % Asegurar que se guarde el histórico
            
            % Ejecutar navegador principal
           navegador_principal_S();  % SIN argumentos


            
            % Cargar resultados
            if exist('resultados_kalman_navegacion.mat', 'file')
                load('resultados_kalman_navegacion.mat');
                informe.resultados_navegador = resultados;
                disp('✅ Resultados del Navegador Principal cargados');
            else
                disp('⚠️ No se encontraron resultados del Navegador Principal');
            end
        end
        
        % Prueba 2: Control Reactivo con Filtro de Kalman
        if ejecutar_reactivo
            disp('🔄 Prueba 2: Ejecutando Control Reactivo con Filtro de Kalman...');
            informe.pruebas_realizadas{end+1} = 'Control Reactivo con Kalman';
            
            % Configurar opciones para la prueba
            opciones_reactivo = struct();
            opciones_reactivo.visualizar = visualizar_pruebas;
            opciones_reactivo.usar_kalman = true;
            
            % Ejecutar control reactivo
            control_reactivo_kalman_S(opciones_reactivo);
            
            % Cargar resultados
            if exist('resultados_control_reactivo_kalman.mat', 'file')
                load('resultados_control_reactivo_kalman.mat');
                informe.resultados_reactivo = resultados;
                disp('✅ Resultados del Control Reactivo cargados');
            else
                disp('⚠️ No se encontraron resultados del Control Reactivo');
            end
        end
        
        % Análisis de Resultados
        if analizar_resultados
            disp('🔄 Analizando Resultados de Validación...');
            
            % Análisis del Navegador Principal
            if ~isempty(informe.resultados_navegador)
                disp('📊 Análisis del Navegador Principal:');
                res_nav = informe.resultados_navegador;
                
                % Calcular métricas clave
                informe.analisis.navegador.error_medio = res_nav.error_medio;
                informe.analisis.navegador.error_max = res_nav.error_max;
                informe.analisis.navegador.num_pasos = size(res_nav.posiciones_estimadas, 1) - 1;
                
                disp(['  - Error medio de estimación: ', num2str(res_nav.error_medio), ' m']);
                disp(['  - Error máximo de estimación: ', num2str(res_nav.error_max), ' m']);
                disp(['  - Número de pasos completados: ', num2str(informe.analisis.navegador.num_pasos)]);
                
                % Analizar convergencia de covarianza
                if isfield(res_nav, 'historico') && ~isempty(res_nav.historico.covarianzas)
                    cov_final = res_nav.historico.covarianzas(end,:);
                    informe.analisis.navegador.covarianza_final = cov_final;
                    disp(['  - Covarianza final (diag): [', num2str(cov_final(1)), ', ', num2str(cov_final(2)), ', ', num2str(cov_final(3)), ']']);
                    
                    % Visualizar convergencia
                    figure;
                    subplot(3,1,1); plot(res_nav.historico.covarianzas(:,1)); title('Convergencia Varianza X'); ylabel('m²'); grid on;
                    subplot(3,1,2); plot(res_nav.historico.covarianzas(:,2)); title('Convergencia Varianza Y'); ylabel('m²'); grid on;
                    subplot(3,1,3); plot(res_nav.historico.covarianzas(:,3)); title('Convergencia Varianza Theta'); ylabel('rad²'); grid on;
                    sgtitle('Convergencia de Covarianza - Navegador Principal');
                end
            end
            
            % Análisis del Control Reactivo
            if ~isempty(informe.resultados_reactivo)
                disp('📊 Análisis del Control Reactivo:');
                res_react = informe.resultados_reactivo;
                
                % Calcular métricas clave
                informe.analisis.reactivo.error_medio = res_react.error_medio;
                informe.analisis.reactivo.error_max = res_react.error_max;
                informe.analisis.reactivo.num_iteraciones = size(res_react.posiciones_estimadas, 1) - 1;
                
                % Calcular distancia final al objetivo
                objetivo = [25, 25]; % Asumiendo el objetivo por defecto
                pos_final_estimada = res_react.posiciones_estimadas(end, 1:2);
                distancia_final_objetivo = sqrt(sum((pos_final_estimada - objetivo).^2));
                informe.analisis.reactivo.distancia_final_objetivo = distancia_final_objetivo;
                
                disp(['  - Error medio de estimación: ', num2str(res_react.error_medio), ' m']);
                disp(['  - Error máximo de estimación: ', num2str(res_react.error_max), ' m']);
                disp(['  - Número de iteraciones: ', num2str(informe.analisis.reactivo.num_iteraciones)]);
                disp(['  - Distancia final al objetivo: ', num2str(distancia_final_objetivo), ' m']);
                
                % Analizar evasión de obstáculos (si hubo)
                if any(min(res_react.distancias_sensores, [], 2) < distancia_seguridad)
                    informe.analisis.reactivo.evasion_activada = true;
                    disp('  - Evasión de obstáculos activada durante la prueba.');
                else
                    informe.analisis.reactivo.evasion_activada = false;
                    disp('  - No se activó la evasión de obstáculos.');
                end
                
                % Analizar convergencia de covarianza
                if isfield(res_react, 'historico') && ~isempty(res_react.historico.covarianzas)
                    cov_final = res_react.historico.covarianzas(end,:);
                    informe.analisis.reactivo.covarianza_final = cov_final;
                    disp(['  - Covarianza final (diag): [', num2str(cov_final(1)), ', ', num2str(cov_final(2)), ', ', num2str(cov_final(3)), ']']);
                    
                    % Visualizar convergencia
                    figure;
                    subplot(3,1,1); plot(res_react.historico.covarianzas(:,1)); title('Convergencia Varianza X'); ylabel('m²'); grid on;
                    subplot(3,1,2); plot(res_react.historico.covarianzas(:,2)); title('Convergencia Varianza Y'); ylabel('m²'); grid on;
                    subplot(3,1,3); plot(res_react.historico.covarianzas(:,3)); title('Convergencia Varianza Theta'); ylabel('rad²'); grid on;
                    sgtitle('Convergencia de Covarianza - Control Reactivo');
                end
            end
        end
        
        % Guardar Informe de Validación
        if guardar_informe
            disp('🔄 Guardando Informe de Validación...');
            save('informe_validacion_sistema.mat', 'informe');
            
            % Generar informe en Markdown
            generar_informe_markdown(informe);
            
            disp('✅ Informe de Validación guardado en informe_validacion_sistema.mat y .md');
        end
        
        disp('=== Validación del Sistema Integrado completada con éxito ===');
        
    catch e
        % Manejo de errores
        disp(['❌ Error durante la validación del sistema: ', e.message]);
        disp('⚠️ Deteniendo validación...');
        
        % Intentar guardar estado actual del informe
        try
            informe.error = struct('mensaje', e.message, 'stack', e.stack);
            save('informe_validacion_sistema_error.mat', 'informe');
            disp('✅ Informe parcial de validación con error guardado');
        catch
            disp('❌ No se pudo guardar el informe parcial de error');
        end
    end
end

% Función para generar informe en formato Markdown
function generar_informe_markdown(informe)
    fid = fopen('informe_validacion_sistema.md', 'w');
    if fid == -1
        disp('⚠️ No se pudo crear el archivo Markdown del informe');
        return;
    end
    
    fprintf(fid, '# Informe de Validación del Sistema Integrado\n\n');
    fprintf(fid, '**Fecha:** %s\n\n', informe.fecha);
    
    fprintf(fid, '## Pruebas Realizadas\n');
    for i = 1:length(informe.pruebas_realizadas)
        fprintf(fid, '- %s\n', informe.pruebas_realizadas{i});
    end
    fprintf(fid, '\n');
    
    % Resultados del Navegador Principal
    if isfield(informe.analisis, 'navegador')
        fprintf(fid, '## Análisis del Navegador Principal\n');
        fprintf(fid, '- **Error medio de estimación:** %.4f m\n', informe.analisis.navegador.error_medio);
        fprintf(fid, '- **Error máximo de estimación:** %.4f m\n', informe.analisis.navegador.error_max);
        fprintf(fid, '- **Número de pasos completados:** %d\n', informe.analisis.navegador.num_pasos);
        if isfield(informe.analisis.navegador, 'covarianza_final')
            cov = informe.analisis.navegador.covarianza_final;
            fprintf(fid, '- **Covarianza final (diag):** [%.4f, %.4f, %.4f]\n', cov(1), cov(2), cov(3));
        end
        fprintf(fid, '\n');
    end
    
    % Resultados del Control Reactivo
    if isfield(informe.analisis, 'reactivo')
        fprintf(fid, '## Análisis del Control Reactivo\n');
        fprintf(fid, '- **Error medio de estimación:** %.4f m\n', informe.analisis.reactivo.error_medio);
        fprintf(fid, '- **Error máximo de estimación:** %.4f m\n', informe.analisis.reactivo.error_max);
        fprintf(fid, '- **Número de iteraciones:** %d\n', informe.analisis.reactivo.num_iteraciones);
        fprintf(fid, '- **Distancia final al objetivo:** %.4f m\n', informe.analisis.reactivo.distancia_final_objetivo);
        if informe.analisis.reactivo.evasion_activada
            fprintf(fid, '- **Evasión de obstáculos:** Activada\n');
        else
            fprintf(fid, '- **Evasión de obstáculos:** No activada\n');
        end
        if isfield(informe.analisis.reactivo, 'covarianza_final')
            cov = informe.analisis.reactivo.covarianza_final;
            fprintf(fid, '- **Covarianza final (diag):** [%.4f, %.4f, %.4f]\n', cov(1), cov(2), cov(3));
        end
        fprintf(fid, '\n');
    end
    
    % Mensaje de error si hubo
    if isfield(informe, 'error')
        fprintf(fid, '## Error Durante la Validación\n');
        fprintf(fid, '**Mensaje:** %s\n', informe.error.mensaje);
        % Podría añadirse más información del stack si es necesario
    end
    
    fclose(fid);
end

% Función auxiliar para obtener campos de estructura con valores por defecto
function valor = getfield_with_default(estructura, campo, valor_default)
    if isfield(estructura, campo)
        valor = estructura.(campo);
    else
        valor = valor_default;
    end
end
