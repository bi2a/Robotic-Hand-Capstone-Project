function [t, datos] = simular_guante()
% SIMULAR_GUANTE  Genera senales falsas que imitan el guante
%
%   t : vector de tiempo (N x 1), 50 Hz, 10 segundos
%   datos : struct con
%             .codo    (N x 1)  angulo de codo [rad]
%             .muneca  (N x 1)  angulo de muneca [rad]
%             .dedos   (N x 6)  [d1j1 d1j2 d2j1 d2j2 d3j1 d3j2] [rad]

% Se le agrega ruido tipo sensor para poder probar filtros despues

deg = pi/180;
fs = 50;                      % frecuencia de muestreo [Hz]
t = (0:1/fs:10)';
N = numel(t);

ruido_deg = 0.5;          % desviacion estandar del ruido [grados]

% Codo: 0 a 130 grados, un ciclo cada ~6.7 s
codo   = 65*(1 - cos(2*pi*0.15*t));
% Muneca: oscila +-40 grados
muneca = 40*sin(2*pi*0.4*t);

% Dedos: cada dedo cierra y abre, con un pequeño desfase entre ellos
dedos = zeros(N, 6);
for i = 1:3
    f = 0.5*(1 - cos(2*pi*0.3*t - (i-1)*0.4));   % 0 (abierto) a 1 (cerrado)
    dedos(:, 2*i-1) = 80*f;    % primera articulacion, joint del dedo
    dedos(:, 2*i) = 90*f;    % segunda articulacion
end

% Agregar ruido de sensor
codo   = codo   + ruido_deg*randn(N,1);
muneca = muneca + ruido_deg*randn(N,1);
dedos  = dedos  + ruido_deg*randn(N,6);

% Respetar limites fisicos y pasar a radianes
datos.codo   = min(max(codo,   0),  130) * deg;
datos.muneca = min(max(muneca, -70), 70) * deg;
datos.dedos  = min(max(dedos,  0),  100) * deg;
end
