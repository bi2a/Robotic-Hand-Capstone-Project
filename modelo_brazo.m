function [arm, dedos, offset_z] = modelo_brazo()
% MODELO_BRAZO  Crea el modelo cinematico del brazo con Robotics Toolbox
%
%   arm : SerialLink de 3 joints (hombro, codo, muneca). El hombro esta
%              arriba (a la altura del "techo") y el brazo cuelga hacia abajo
%   dedos : cell array {1x3}, cada dedo es un SerialLink de 2 joints.
%   offset_z : posicion lateral de cada dedo respecto a la palma [m].

% Todas las longitudes estan en metros y son PROVISIONALES: cuando
% tengamos el diseno 3D definitivo, solo hay que cambiar esta seccion

% Convencion de signos (todos positivos = flexion hacia ADELANTE, +X):
%   q = 0 en todos los joints -> brazo estirado colgando hacia abajo.

deg = pi/180;

%% Parametros editables 
altura_hombro = 0.80;   % altura del hombro respecto al piso
L_brazo       = 0.30;   % hombro -> codo
L_antebrazo   = 0.25; % codo -> muneca
L_mano        = 0.10;   % muneca -> base de los dedos (palma)
L_falange1    = 0.045;  % dedo, primera falange
L_falange2    = 0.030;  % dedo, segunda falange
sep_dedos     = 0.025;  % separacion entre dedos (perpendicular al plano de flexion)

%% Brazo: hombro + codo + muneca
%Link([theta d a alpha]) en convencion DH estandar (theta = variable).
A(1) = Link([0 0 L_brazo     0], 'standard');   % hombro (flexion/extension)
A(2) = Link([0 0 L_antebrazo 0], 'standard');   % codo
A(3) = Link([0 0 L_mano      0], 'standard');   % muneca (flexion/extension)

%Limites aproximados de un brazo humano (rango articular tipico)
A(1).qlim = [-60  180]*deg;   % hombro: extension 60, flexion 180
A(2).qlim = [   0 145]*deg;   % codo: 0 a ~145 (no hiperextiende)
A(3).qlim = [ -70  80]*deg;   % muneca: extension 70, flexion 80

arm = SerialLink(A, 'name', 'brazo');

% Base en el hombro, a altura_hombro, orientada para que con q = 0 el
%brazo cuelgue hacia abajo (-Z) y la flexion positiva vaya hacia +X.
R_base   = [0 1 0; 0 0 -1; -1 0 0];
arm.base = [R_base, [0; 0; altura_hombro]; 0 0 0 1];   % matriz homogenea 4x4

%% Dedos: 3 dedos x 2 joints
dedos = cell(1, 3);
for i = 1:3
    F(1) = Link([0 0 L_falange1 0], 'standard');
    F(2) = Link([0 0 L_falange2 0], 'standard');

    F(1).qlim = [0  90]*deg;    % articulacion metacarpofalangica
    F(2).qlim = [0 100]*deg;    % articulacion interfalangica proximal

    dedos{i} = SerialLink(F, 'name', sprintf('dedo%d', i));
end

% La base de cada dedo se actualiza en main.m en cada cuadro,
% porque depende de donde quede la palma.
offset_z = [-1 0 1] * sep_dedos;
end
