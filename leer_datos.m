function [q_codo, q_muneca, q_dedos, N] = leer_datos(k)
% LEER_DATOS  Devuelve la muestra k de los angulos del guante.
%
%   q_codo   : escalar [rad]
%   q_muneca : escalar [rad]
%   q_dedos  : 1x6 [rad]  [d1j1 d1j2 d2j1 d2j2 d3j1 d3j2]
%   N : numero total de muestras (solo vale para datos simulados)
%
% HOY: lee de simular_guante().
% DESPUES: aca se reemplaza por la lectura del puerto serial del ESP32.
% El resto del programa (main.m) no necesita cambiar

persistent datos Ntotal
if isempty(datos)
    [~, datos] = simular_guante();
    Ntotal = numel(datos.codo);
end

q_codo   = datos.codo(k);
q_muneca = datos.muneca(k);
q_dedos  = datos.dedos(k, :);
N = Ntotal;
end
