function y = esp32_link(u)
% ESP32_LINK  wireless link between Simulink / MATLAB and the robot ESP32.
%
%   esp32_link('open')   opens the link
%   y = esp32_link(u)    u = [wL_ref wR_ref mode V_test Kp Ki]
%                        sends   "S,wL_ref,wR_ref,mode,V_test,Kp,Ki"
%                        returns y = [wL; wR; VL; VR]  (newest "D,..." packet from the ESP32)
%   esp32_link('count')  number of packets received so far
%   esp32_link('close')  stops the motors and closes the link
%
% WiFi (default): the ESP32 makes its own WiFi network "Robot_ESP32" (password 12345678).
%   Connect the laptop to it, the robot is at 192.168.4.1, UDP port 4210.
%   Uses udpport (base MATLAB R2020b+, no toolbox).
% Bluetooth (optional): set ESP32_LINK = "bt" and ESP32_PORT = "COMx" in the workspace
%   and generate the ESP32 code with LINK = 'bt'.

persistent h last npk mode_
if isempty(last), last = zeros(4,1); end
if isempty(npk),  npk = 0; end

if ischar(u) || isstring(u)
    switch lower(char(u))
        case 'open'
            if isempty(h), [h, mode_] = open_link(); end
        case 'count'
            y = npk;  return
        case 'close'
            if ~isempty(h)
                for k = 1:3, send_line(h, mode_, "S,0,0,1,0,0,0"); pause(0.02); end
                try, delete(h.obj); catch, end
            end
            h = []; last = zeros(4,1); npk = 0;
            disp('ESP32 link closed (motors stopped)');
    end
    y = last;
    return
end

if isempty(h), [h, mode_] = open_link(); end

% ---- send the command
send_line(h, mode_, sprintf('S,%.4f,%.4f,%d,%.3f,%.4f,%.4f', u(1), u(2), round(u(3)), u(4), u(5), u(6)));

% ---- read everything that arrived, keep the newest packet
lines = read_lines(h, mode_);
for k = 1:numel(lines)
    ln = strtrim(lines{k});
    if numel(ln) > 2 && strncmp(ln, 'D,', 2)
        v = str2double(strsplit(ln(3:end), ','));      % t_ms,wL,wR,VL,VR,cntL,cntR
        if numel(v) >= 5 && all(~isnan(v(1:5)))
            last = v(2:5).';  npk = npk + 1;
        end
    end
end
y = last(:);
end

% =====================================================================================
function [h, mode_] = open_link()
mode_ = "wifi";
try, mode_ = lower(string(evalin('base', 'ESP32_LINK'))); catch, end
if mode_ == "bt"
    port = "COM5";  try, port = evalin('base', 'ESP32_PORT'); catch, end
    fprintf('Opening Bluetooth %s ...\n', port);
    h.obj = serialport(port, 115200, 'Timeout', 0.05);
    configureTerminator(h.obj, "LF");  flush(h.obj);
else
    ip = "192.168.4.1";  try, ip = evalin('base', 'ESP32_IP'); catch, end
    h.obj  = udpport("datagram", "IPV4", "Timeout", 0.05);
    h.ip   = char(ip);
    h.port = 4210;
    fprintf('WiFi link to the robot at %s:%d (laptop must be on WiFi "Robot_ESP32")\n', h.ip, h.port);
end
end

function send_line(h, mode_, txt)
try
    if mode_ == "bt"
        writeline(h.obj, txt);
    else
        write(h.obj, uint8(char(txt)), "uint8", h.ip, h.port);
    end
catch err
    warning('esp32link:write', 'send failed: %s', err.message);
end
end

function lines = read_lines(h, mode_)
lines = {};
try
    if mode_ == "bt"
        while h.obj.NumBytesAvailable > 0
            lines{end+1} = char(readline(h.obj)); %#ok<AGROW>
        end
    else
        n = h.obj.NumDatagramsAvailable;
        if n > 0
            d = read(h.obj, n, "uint8");
            for k = 1:numel(d)
                parts = strsplit(char(d(k).Data(:).'), newline);
                lines = [lines, parts]; %#ok<AGROW>
            end
        end
    end
catch
end
end
