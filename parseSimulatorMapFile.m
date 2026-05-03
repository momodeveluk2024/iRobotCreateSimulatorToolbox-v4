function [walls, lines, beacons, virtualWalls] = parseSimulatorMapFile(filename)
% parseSimulatorMapFile Read a simulator map text file without opening the GUI.

walls = [];
lines = [];
beacons = {};
virtualWalls = [];

fid = fopen(filename, 'r');
if fid < 0
    error('Simulator:mapOpenFailed', 'Could not open map file: %s', filename);
end
cleanup = onCleanup(@() fclose(fid));

while ~feof(fid)
    rawLine = fgetl(fid);
    if ~ischar(rawLine)
        continue
    end
    line = lower(strtrim(rawLine));
    commentIdx = strfind(line, '%');
    if ~isempty(commentIdx)
        line = strtrim(line(1:commentIdx(1)-1));
    end
    if isempty(line)
        continue
    end

    parts = strsplit(line);
    switch parts{1}
        case 'wall'
            if numel(parts) == 5
                values = str2double(parts(2:5));
                if all(isfinite(values))
                    walls = [walls; values];
                end
            end
        case 'line'
            if numel(parts) == 5
                values = str2double(parts(2:5));
                if all(isfinite(values))
                    lines = [lines; values];
                end
            end
        case 'beacon'
            values = sscanf(line, 'beacon %f %f [%f %f %f] %s');
            tokens = regexp(line, 'beacon\s+([-+.\deE]+)\s+([-+.\deE]+)\s+\[([-+.\deE]+)\s+([-+.\deE]+)\s+([-+.\deE]+)\]\s+(\S+)', 'tokens', 'once');
            if ~isempty(tokens)
                beacons = [beacons; {str2double(tokens{1}) str2double(tokens{2}) ...
                    str2double(tokens{3}) str2double(tokens{4}) str2double(tokens{5}) tokens{6}}];
            elseif numel(values) >= 5
                beacons = [beacons; {values(1) values(2) values(3) values(4) values(5) ''}];
            end
        case 'virtwall'
            if numel(parts) >= 4
                values = str2double(parts(2:min(5, numel(parts))));
                if numel(values) == 3
                    values(4) = 1;
                end
                if numel(values) >= 4 && all(isfinite(values(1:4)))
                    virtualWalls = [virtualWalls; values(1:4)];
                end
            end
    end
end
end
