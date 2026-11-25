% MAIN  Anima el brazo (hombro fijo + codo + muneca + 3 dedos) con los
% datos del guante.
%
% Ejecutar ESTE archivo. Los otros (modelo_brazo.m, simular_guante.m,
% leer_datos.m) deben estar en la misma carpeta.
%
% Requiere el Robotics Toolbox (startup_rvc) solo para definir los eslabones.
% El DIBUJO se hace con plot3 normal de MATLAB (no con robot.plot), para
% no depender de como el toolbox maneja la base ni varios robots a la vez.

clear; clear functions; clc; close all;

%% 1. Modelo
[arm, dedos, offset_z] = modelo_brazo();
T_base = SE3(arm.base);                  % arm.base es una matriz 4x4: la paso a SE3
H = T_base.t(3);                         % altura del hombro
fprintf('Altura del hombro (arm.base): z = %.2f m  (debe ser 0.80)\n', H);

%% 2. Figura
fig = figure('Name', 'Simulacion brazo-guante', 'Color', 'w');
ax  = axes(fig); hold(ax, 'on'); grid(ax, 'on');
axis(ax, [-0.5 0.6  -0.3 0.3  0 0.9]);
daspect(ax, [1 1 1]);
xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]'); zlabel(ax, 'Z [m]');
view(ax, 40, 15);       % vista lateral pura: view(ax, 0, 0)

% "Techo" donde se fija el hombro
fill3(ax, [-0.3 0.3 0.3 -0.3], [-0.3 -0.3 0.3 0.3], (H + 0.01)*ones(1,4), ...
      [0.6 0.6 0.6], 'FaceAlpha', 0.4, 'EdgeColor', 'none');

% Lineas que se actualizan en cada cuadro
h_arm = plot3(ax, 0, 0, 0, '-o', 'LineWidth', 7, 'Color', [0.1 0.3 0.8], ...
              'MarkerSize', 10, 'MarkerFaceColor', 'k');
h_dedo = gobjects(1, 3);
for i = 1:3
    h_dedo(i) = plot3(ax, 0, 0, 0, '-o', 'LineWidth', 3, ...
                      'Color', [0.85 0.2 0.2], 'MarkerSize', 5, ...
                      'MarkerFaceColor', 'k');
end

%% 3. Bucle principal
[~, ~, ~, N] = leer_datos(1);

for k = 1:N
    [q_codo, q_muneca, q_dedos] = leer_datos(k);

    q_hombro = 0;               % aun no hay sensor para el hombro
    q_arm = [q_hombro, q_codo, q_muneca];

    [P_arm, P_dedos] = cinematica(arm, dedos, offset_z, q_arm, q_dedos);

    set(h_arm, 'XData', P_arm(1,:), 'YData', P_arm(2,:), 'ZData', P_arm(3,:));
    for i = 1:3
        set(h_dedo(i), 'XData', P_dedos{i}(1,:), ...
                       'YData', P_dedos{i}(2,:), ...
                       'ZData', P_dedos{i}(3,:));
    end

    title(ax, sprintf('t = %.2f s | codo = %.0f deg | muneca = %.0f deg', ...
          (k-1)*0.02, q_codo*180/pi, q_muneca*180/pi));
    drawnow;
    pause(0.02);                         % ~50 cuadros por segundo
end

% Funciones locales
function [P_arm, P_dedos] = cinematica(arm, dedos, offset_z, q_arm, q_dedos)
    % Devuelve las posiciones 3D de cada articulacion.
    %  P_arm   : 3x4  [hombro, codo, muneca, punta de la mano]
    %  P_dedos : cell{1x3}, cada una 3x3 [base, articulacion 1, punta]
    T0 = SE3(arm.base);
    T1 = T0 * arm.links(1).A(q_arm(1));
    T2 = T1 * arm.links(2).A(q_arm(2));
    T3 = T2 * arm.links(3).A(q_arm(3));
    P_arm = [T0.t, T1.t, T2.t, T3.t];

    P_dedos = cell(1, 3);
    for i = 1:3
        Tf0 = T3  * SE3(0, 0, offset_z(i));
        Tf1 = Tf0 * dedos{i}.links(1).A(q_dedos(2*i-1));
        Tf2 = Tf1 * dedos{i}.links(2).A(q_dedos(2*i));
        P_dedos{i} = [Tf0.t, Tf1.t, Tf2.t];
    end
end
