function [E1, E2, E3] = Angulos_Euler(q1, q2, q3, q4)

Tensor_Inercia = diag([150, 80, 120]); % Tensor de inercia del CubeSat.

Cuaternion_Inicial = [q1, q2, q3, q4];

Vel_Angular_Inicial = [0;0;0]; % Velocidad angular inicial (asumida cero).

Cuaternion_Deseado = [1, 0, 0, 0]; % Cuaternión que representa la orientación deseada (en este caso, apuntar al nadir).

% Inicialización de los términos integrales para ambos lazos de control.

Error_Integral_Actitud = [0;0;0];
Error_Integral_Vel_Angular = [0;0;0];

dt = 0.05; % Tiempo de muestreo

% Error de actitud

Error_Actitud = quatmultiply(quatconj(Cuaternion_Deseado), Cuaternion_Inicial);

% Controlador encargado de determinar la velocidad angular deseada a partir del error de actitud.

Velocidad_Angular_Deseada = PID_Control_Orientacion(Error_Actitud, Error_Integral_Actitud, dt);

% Error de velocidad angular

Error_Velocidad_Angular = Velocidad_Angular_Deseada - Vel_Angular_Inicial;

% Controlador encargado de calcular los momentos de cuerpo (torques) que permiten corregir el error de velocidad angular.

Momentos_De_Cuerpo = PID_Control_Velocidad_Angular(Error_Velocidad_Angular, Error_Integral_Vel_Angular, dt);

% Actualización de la velocidad angular del CubeSat.

Cambio_Velocidad_Angular = Tensor_Inercia \ (Momentos_De_Cuerpo - cross(Vel_Angular_Inicial, Tensor_Inercia*Vel_Angular_Inicial));
Vel_Angular_Inicial = Vel_Angular_Inicial + Cambio_Velocidad_Angular*dt;

% Actualización y normalización del cuaternión.

Cambio_Cuaternion = 0.5*quatmultiply([0,Vel_Angular_Inicial'], Cuaternion_Inicial);
Cuaternion_Inicial = Cuaternion_Inicial + Cambio_Cuaternion*dt;
Cuaternion_Inicial = Cuaternion_Inicial/norm(Cuaternion_Inicial);

% El cuaternión actualizado se convierte a ángulos de Euler en el orden ZYX.

Angulos_Euler = quat2eul(Cuaternion_Inicial, 'ZYX'); 

% Convertir la posición angular a grados.

Posicion_Angular = rad2deg(Angulos_Euler);
E1 = Posicion_Angular(1);
E2 = Posicion_Angular(2);
E3 = Posicion_Angular(3);

end

% Función del controlador encargado de ajustar la orientación del satélite.

function Velocidad_Angular_Deseada = PID_Control_Orientacion(Error_Actitud, Error_Integral_Actitud, dt)

% Constantes del controlador PID que ajusta la orientación del satélite.

Kp_Orientacion = 0.0025; % Ganancia proporcional para el control de la orientación.
Ki_Orientacion = 0.0023; % Ganancia integral para el control de la orientación.
Kd_Orientacion = 0.11; % Ganancia derivativa para el control de la orientación.

Angulo_Doble = acos(Error_Actitud(1));
Seno_Angulo_Doble = sin(Angulo_Doble);

if Seno_Angulo_Doble < eps
    Seno_Angulo_Doble = eps;
end

Error_Velocidad_Angular = Error_Actitud(2:4)'/Seno_Angulo_Doble;

Error_Integral_Actitud = Error_Integral_Actitud + Error_Velocidad_Angular*dt;
Error_Derivativo = Error_Velocidad_Angular/dt;
Velocidad_Angular_Deseada = Kp_Orientacion * Error_Velocidad_Angular + Ki_Orientacion*Error_Integral_Actitud + Kd_Orientacion*Error_Derivativo;

end

% Función del controlador encargado de ajustar la velocidad angular del CubeSat.

function Momentos_De_Cuerpo = PID_Control_Velocidad_Angular(Error_Velocidad_Angular, Error_Integral_Vel_Angular, dt)

% Constantes del controlador PID que ajusta la velocidad angular del satélite. 

Kp_Vel_Angular = 0.012; % Ganancia proporcional para el control de la velocidad angular.
Ki_Vel_Angular = 0.018; % Ganancia integral para el control de la velocidad angular.
Kd_Vel_Angular = 0.14; % Ganancia derivativa para el control de la velocidad angular.

Error_Integral_Vel_Angular = Error_Integral_Vel_Angular + Error_Velocidad_Angular*dt;
Error_Derivativo = Error_Velocidad_Angular/dt;
Momentos_De_Cuerpo = Kp_Vel_Angular*Error_Velocidad_Angular + Ki_Vel_Angular*Error_Integral_Vel_Angular + Kd_Vel_Angular*Error_Derivativo;

end

% Funciones que permiten hallar el conjugado de un cuaternión y multiplicar
% dos cuaterniones.

function q_conj = quatconj(q)
q_conj = [q(1), -q(2), -q(3), -q(4)];
end

function q_out = quatmultiply(q,r)

q_out = [q(1)*r(1) - q(2)*r(2) - q(3)*r(3) - q(4)*r(4)... 
         q(1)*r(2) + q(2)*r(1) + q(3)*r(4) - q(4)*r(3)...
         q(1)*r(3) - q(2)*r(4) + q(3)*r(1) + q(4)*r(2)...
         q(1)*r(4) - q(2)*r(3) - q(3)*r(2) - q(4)*r(1)];

end