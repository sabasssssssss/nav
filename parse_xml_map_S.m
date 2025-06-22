function [vertices, balizas] = parse_xml_map_S(xml_path)
    % Función para parsear un archivo XML de mapa y extraer vértices de obstáculos y balizas.
    % xml_path: Ruta al archivo XML.
    % vertices: Matriz de Nx2 con las coordenadas [x, y] de los vértices de los obstáculos.
    % balizas: Matriz de Mx2 con las coordenadas [x, y] de las balizas.

    % Leer el contenido del archivo XML
    xml_content = fileread(xml_path);

    % --- Extracción de Vértices de Obstáculos ---
    vertices = [];
    % Expresión regular mejorada para encontrar cualquier <vertex> ... </vertex> en el XML
    vertex_pattern = '<vertex>\s*([\s\S]*?)\s*</vertex>';
    vertex_tokens = regexp(xml_content, vertex_pattern, 'tokens');

    if ~isempty(vertex_tokens)
        for i = 1:length(vertex_tokens)
            % Extraer cada par de coordenadas {x,y,z}
            coord_pattern = '\{([\d\.-]+),\s*([\d\.-]+),\s*([\d\.-]+)\}';
            coords = regexp(vertex_tokens{i}{1}, coord_pattern, 'tokens');
            for j = 1:length(coords)
                x = str2double(coords{j}{1});
                y = str2double(coords{j}{2});
                % Ignoramos z por ahora, ya que estamos trabajando en 2D para el mapa de ocupación
                vertices = [vertices; x, y];
            end
        end
    end

    % --- Extracción de Balizas ---
    balizas = [];
    % Expresión regular para encontrar balizas con la etiqueta <LandMark> y sus posiciones.
    % Busca <LandMark ...> y luego <position> {x,y,z} </position>
    landmark_pattern = '<LandMark[^>]*?(?:mark_id="(\d+)")?[^>]*?>\s*<position>\s*\{([\d.-]+),([\d.-]+),([\d.-]+)\}\s*</position>\s*</LandMark>';
    landmark_tokens = regexp(xml_content, landmark_pattern, 'tokens');

    if ~isempty(landmark_tokens)
        for i = 1:length(landmark_tokens)
            % Si hay un mark_id, úsalo, de lo contrario, ignóralo o asigna un valor predeterminado
            if ~isempty(landmark_tokens{i}{1})
                % Si el primer token es el mark_id (cuando existe)
                x = str2double(landmark_tokens{i}{2});
                y = str2double(landmark_tokens{i}{3});
            else
                % Si no hay mark_id, los tokens se desplazan
                x = str2double(landmark_tokens{i}{1});
                y = str2double(landmark_tokens{i}{2});
            end

            % Filtrar coordenadas válidas (ejemplo: dentro de un rango razonable del mapa)
            % Asumiendo que los mapas están en un rango de 0 a 35 o similar
            if x >= -50 && x <= 50 && y >= -50 && y <= 50 % Rango amplio para cubrir ambos mapas
                balizas = [balizas; x, y];
            end
        end
    end

    if isempty(vertices)
        disp('⚠️ No se encontraron vértices de obstáculos en el XML.');
    end
    if isempty(balizas)
        disp('⚠️ No se encontraron balizas en el XML.');
    end

