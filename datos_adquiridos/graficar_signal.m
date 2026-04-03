% graficar_señal.m
% Lee el archivo y grafica cada bloque JSON por separado
% Presionar Enter para pasar al siguiente bloque, q+Enter para salir

clc; clear; close all;

filename = 'datos.txt';

%% Leer y parsear bloques JSON
fid = fopen(filename, 'r');
if fid == -1
    error('No se puede abrir el archivo: %s', filename);
end

bloques = {};

while ~feof(fid)
    linea = strtrim(fgetl(fid));
    
    if contains(linea, '"Muestras"')
        % Extraer timestamp
        ts_match = regexp(linea, '"ts":(\d+:\d+:\d+)', 'tokens');
        ts = '';
        if ~isempty(ts_match)
            ts = ts_match{1}{1};
        end
        
        % Extraer numeros
        datos_str = regexp(linea, '"Muestras":\s*(.*)', 'tokens');
        if ~isempty(datos_str)
            nums = str2num(strrep(strrep(datos_str{1}{1}, '}', ''), ',', ' ')); %#ok<ST2NM>
            if ~isempty(nums)
                bloque.ts = ts;
                bloque.muestras = nums;
                bloques{end+1} = bloque;
            end
        end
    end
end
fclose(fid);

fprintf('Se encontraron %d bloques.\n\n', length(bloques));

%% Graficar bloque por bloque
figure(1);

for i = 1:length(bloques)
    b = bloques{i};
    
    clf;
    plot(b.muestras, 'b-', 'LineWidth', 1);
    title(sprintf('Bloque %d/%d  |  ts: %s  |  %d muestras', ...
        i, length(bloques), b.ts, length(b.muestras)));
    xlabel('Muestra');
    ylabel('Valor ADC');
    ylim([0 16384]);
    grid on;
    drawnow;
    
    if i < length(bloques)
        fprintf('Bloque %d/%d [ts=%s] - Enter para continuar, q+Enter para salir: ', ...
            i, length(bloques), b.ts);
        r = input('', 's');
        if strcmpi(r, 'q')
            break;
        end
    else
        fprintf('Ultimo bloque (%d/%d). Listo.\n', i, length(bloques));
    end
end
