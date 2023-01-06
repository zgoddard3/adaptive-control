close all

m = 4.25;
d = 0.1;
g = 9.8;
r = 0.26;
J = 0.0475;

tau0 = m*g;

A = [0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, -tau0/m, 0, 0;
     1, 0, 0, -d/m, 0, 0;
     0, 1, 0, 0, -d/m, 0;
     0, 0, 1, 0, 0, 0]';

B = [0, 0;
     0, 0;
     0, 0;
     1/m, 0;
     0, 1/m;
     -r/J, 0];

Q = diag([.1, 1, 1, 1, 1, 1]);
[P, K, L] = icare(A, B, Q, [], [], [], []);

% linear_control = @(s, r)inv(B'*B)*B'*(B*K-A)*r-K*s + [tau0; 0];

tspan = [0, 50];
x0 = [0, 0, 0, 0, 0, 0]';

% ====================== Step Response =============================
ref = @(t)[1, 1, 0, 0, 0, 0, 0, 0, 0]';
odefun = @(t,s)odefun1(t,s,linearControl(s, ref(t)));
[t, x] = ode45(odefun, tspan, x0);
r = zeros(size(x));
for i = 1:length(t)
    rt = ref(t(i));
    r(i,:) = rt(1:6);
end

plotStates(t,x,r,"LinearControlStep")
sgtitle("Linear Control Step Response")

% ======================== Sinusoid Response =============================
ref = @(t)[sin(t), cos(t), 0, cos(t), -sin(t), 0, -sin(t), -cos(t), 0]';
odefun = @(t,s)odefun1(t,s,linearControl(s, ref(t)));
[t, x] = ode45(odefun, tspan, x0);
r = zeros(size(x));
for i = 1:length(t)
    rt = ref(t(i));
    r(i,:) = rt(1:6);
end

plotStates(t,x,r, "LinearControlSinusoid")
sgtitle("Linear Control Sinusoid Response")

%=========================================================================
% ========================= With Uncertainty =============================
%=========================================================================


% ====================== Step Response =============================
ref = @(t)[1, 1, 0, 0, 0, 0, 0, 0, 0]';
odefun = @(t,s)odefun2(t,s,linearControl(s, ref(t)));
[t, x] = ode45(odefun, tspan, x0);
r = zeros(size(x));
for i = 1:length(t)
    rt = ref(t(i));
    r(i,:) = rt(1:6);
end

plotStates(t,x,r,"LinearControlStepWithUncertainty")
sgtitle("Linear Control Step Response w/ Uncertainty")

dfig1 = figure;
hold on
d = sqrt(sum((x(:,1:2) - r(:,1:2)).^2,2));
plot(t, d, 'linewidth',2);

% ======================== Sinusoid Response =============================
ref = @(t)[sin(t), cos(t), 0, cos(t), -sin(t), 0, -sin(t), -cos(t), 0]';
odefun = @(t,s)odefun2(t,s,linearControl(s, ref(t)));
[t, x] = ode45(odefun, tspan, x0);
r = zeros(size(x));
for i = 1:length(t)
    rt = ref(t(i));
    r(i,:) = rt(1:6);
end

plotStates(t,x,r,"LinearControlSinusoidWithUncertainty")
sgtitle("Linear Control Sinusoid Response w/ Uncertainty")

dfig2 = figure;
hold on
d = sqrt(sum((x(:,1:2) - r(:,1:2)).^2,2));
plot(t, d, 'linewidth',2);

%=========================================================================
% ========================= Adaptive System =============================
%=========================================================================

Am = A - B*K;
Bm = B;

kx0 = (B'*B)\B'*(Am - A);
kr0 = (B'*B)\B'*Bm;
alpha0 = -[0, tau0]';

x0 = [zeros(12,1); kx0(:); kr0(:); alpha0(:)];


% ====================== Step Response =============================
ref = @(t)[1, 1, 0, 0, 0, 0, 0, 0, 0]';
odefun = @(t,s)odefun5(t,s,ref(t));
[t, x] = ode45(odefun, tspan, x0);
r = zeros(length(t), 6);
for i = 1:length(t)
    rt = ref(t(i));
    r(i,:) = rt(1:6);
end

plotAdaptiveStates2(t,x,r, "Adaptive Control Step Response")

figure(dfig1)
hold on
d = sqrt(sum((x(:,1:2) - r(:,1:2)).^2,2));
plot(t, d, 'linewidth',2)

% ======================== Sinusoid Response =============================
ref = @(t)[sin(t), cos(t), 0, cos(t), -sin(t), 0, -sin(t), -cos(t), 0]';
odefun = @(t,s)odefun5(t,s,ref(t));
[t, x] = ode45(odefun, tspan, x0);
r = zeros(length(t), 6);
for i = 1:length(t)
    rt = ref(t(i));
    r(i,:) = rt(1:6);
end

plotAdaptiveStates2(t,x,r, "Adaptive Control Sinusoid Response")

figure(dfig2)
hold on
d = sqrt(sum((x(:,1:2) - r(:,1:2)).^2,2));
plot(t, d, 'linewidth',2)

% =======================================================================
% =========================== Post Adaptation =====================+=====
% =======================================================================

kx0 = reshape(x(end, 13:24), [2, 6]);
kr0 = reshape(x(end, 25:28), [2,2]);
alpha0 = reshape(x(end, 29:end), [2,1]);

x0 = [zeros(12,1); kx0(:); kr0(:); alpha0(:)];

% ====================== Step Response =============================
ref = @(t)[1, 1, 0, 0, 0, 0, 0, 0, 0]';
odefun = @(t,s)odefun5(t,s,ref(t));
[t, x] = ode45(odefun, tspan, x0);
r = zeros(length(t), 6);
for i = 1:length(t)
    rt = ref(t(i));
    r(i,:) = rt(1:6);
end

plotAdaptiveStates2(t,x,r, "Adaptive Control Step Response Post-Adaptation")

figure(dfig1)
hold on
d = sqrt(sum((x(:,1:2) - r(:,1:2)).^2,2));
plot(t, d, 'linewidth',2)


% ======================== Sinusoid Response =============================
ref = @(t)[sin(t), cos(t), 0, cos(t), -sin(t), 0, -sin(t), -cos(t), 0]';
odefun = @(t,s)odefun5(t,s,ref(t));
[t, x] = ode45(odefun, tspan, x0);
r = zeros(length(t), 6);
for i = 1:length(t)
    rt = ref(t(i));
    r(i,:) = rt(1:6);
end

plotAdaptiveStates2(t,x,r, "Adaptive Control Sinusoid Response Post-Adaptation")

figure(dfig2)
hold on
d = sqrt(sum((x(:,1:2) - r(:,1:2)).^2,2));
plot(t, d, 'linewidth',2)

%=========================================================================
% ==================== Nonlinear Adaptive System ========================
%=========================================================================

Am = A - B*K;
Bm = B;

kx0 = (B'*B)\B'*(Am - A);
kr0 = (B'*B)\B'*Bm;
alpha0 = -[m*g, 0;0, m*g];

x0 = [zeros(12,1); kx0(:); kr0(:);alpha0(:)];

% ====================== Step Response =============================
ref = @(t)[1, 1, 0, 0, 0, 0, 0, 0, 0]';
odefun = @(t,s)odefun4(t,s,ref(t));
[t, x] = ode45(odefun, tspan, x0);
r = zeros(length(t), 6);
for i = 1:length(t)
    rt = ref(t(i));
    r(i,:) = rt(1:6);
end

plotAdaptiveStates2(t,x,r, "Adaptive Nonlinear Control Step Response")

figure(dfig1)
hold on
d = sqrt(sum((x(:,1:2) - r(:,1:2)).^2,2));
plot(t, d, 'linewidth',2);


% ======================== Sinusoid Response =============================
ref = @(t)[sin(t), cos(t), 0, cos(t), -sin(t), 0, -sin(t), -cos(t), 0]';
odefun = @(t,s)odefun4(t,s,ref(t));
[t, x] = ode45(odefun, tspan, x0);
r = zeros(length(t), 6);
for i = 1:length(t)
    rt = ref(t(i));
    r(i,:) = rt(1:6);
end

plotAdaptiveStates2(t,x,r, "Adaptive Nonlinear Control Sinusoid Response")

figure(dfig2)
hold on
d = sqrt(sum((x(:,1:2) - r(:,1:2)).^2,2));
plot(t, d, 'linewidth',2);

% =======================================================================
% =========================== Post Adaptation =====================+=====
% =======================================================================

kx0 = reshape(x(end, 13:24), [2, 6]);
kr0 = reshape(x(end, 25:28), [2,2]);
alpha0 = reshape(x(end, 29:end), [2,2]);

x0 = [zeros(12,1); kx0(:); kr0(:); alpha0(:)];

% ====================== Step Response =============================
ref = @(t)[1, 1, 0, 0, 0, 0, 0, 0, 0]';
odefun = @(t,s)odefun4(t,s,ref(t));
[t, x] = ode45(odefun, tspan, x0);
r = zeros(length(t), 6);
for i = 1:length(t)
    rt = ref(t(i));
    r(i,:) = rt(1:6);
end

plotAdaptiveStates2(t,x,r, "Adaptive Nonlinear Control Step Response Post-Adaptation")

figure(dfig1)
hold on
d = sqrt(sum((x(:,1:2) - r(:,1:2)).^2,2));
plot(t, d, 'linewidth',2)
xlabel('Time [s]')
ylabel('Distance [m]')
dfig1.Children.FontSize = 18;
legend('Linear', 'Adaptive', 'Post-Adaptation', 'Non-Linear', 'NL Post-Adaptation')

saveas(dfig1, "Figures/PNG/StepDistance", "png")


% ======================== Sinusoid Response =============================
ref = @(t)[sin(t), cos(t), 0, cos(t), -sin(t), 0, -sin(t), -cos(t), 0]';
odefun = @(t,s)odefun4(t,s,ref(t));
[t, x] = ode45(odefun, tspan, x0);
r = zeros(length(t), 6);
for i = 1:length(t)
    rt = ref(t(i));
    r(i,:) = rt(1:6);
end

plotAdaptiveStates2(t,x,r, "Adaptive Nonlinear Control Sinusoid Response Post-Adaptation")

figure(dfig2)
hold on
d = sqrt(sum((x(:,1:2) - r(:,1:2)).^2,2));
plot(t, d, 'linewidth',2)
xlabel('Time [s]')
ylabel('Distance [m]')
dfig2.Children.FontSize = 18;
legend('Linear', 'Adaptive', 'Post-Adaptation', 'Non-Linear', 'NL Post-Adaptation')

saveas(dfig2, "Figures/PNG/SinusoidDistance", "png")


function u = linearControl(x,xd)
m = 4.25;
d = 0.1;
g = 9.8;
r = 0.26;
J = 0.0475;

tau0 = m*g;

A = [0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, -tau0/m, 0, 0;
     1, 0, 0, -d/m, 0, 0;
     0, 1, 0, 0, -d/m, 0;
     0, 0, 1, 0, 0, 0]';

B = [0, 0;
     0, 0;
     0, 0;
     1/m, 0;
     0, 1/m;
     -r/J, 0];

% K = [0.3162,0,-6.4109,1.1748,0,-1.7795;
%     0,1.4142,0,0,3.5098,0];
K = [0.3162,0,-6.4109,1.1748,0,-1.7795;
    0,1,0,0,2.9838,0];

xddot = xd(4:end);
xd = xd(1:6);

r = (B'*B)\B'*(xddot+(B*K-A)*xd);
f = r-K*x+[0;tau0];
u = [norm(f); atan2(f(1),f(2))];
end

function sdot = odefun1(t, s, u)

m = 4.25;
d = 0.1;
g = 9.8;
r = 0.26;
J = 0.0475;

theta = s(3);
qdot = s(4:5);
thetadot = s(6);

tau = u(1);
psi = u(2);

psi = max(min(psi, pi/3), -pi/3);

e2 = rot(psi);
e2 = e2(:,2);

sdot = zeros(6,1);
sdot(1:2) = qdot;
sdot(3) = thetadot;
sdot(4:5) = (-d.*qdot + rot(theta)*e2.*tau)./m - [0;g];
sdot(6) = -r*sin(psi)*tau/J;
end

function sdot = odefun2(t, s, u)

m = 4.25 + 0.3571;
d = 0.1 + 0.0182;
g = 9.81;
r = 0.26 - 0.0318;
J = 0.0475 - 0.0082;

theta = s(3);
qdot = s(4:5);
thetadot = s(6);

tau = u(1);
psi = u(2);

psi = max(min(psi, pi/3), -pi/3);

e2 = rot(psi);
e2 = e2(:,2);

sdot = zeros(6,1);
sdot(1:2) = qdot;
sdot(3) = thetadot;
sdot(4:5) = (-d.*qdot + rot(theta)*e2.*tau)./m - [0;g];
sdot(6) = -r*sin(psi)*tau/J;
end

function sdot = odefun3(t, s, xd)

m = 4.25;
d = 0.1;
g = 9.8;
r = 0.26;
J = 0.0475;

tau0 = m*g;

A = [0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, -tau0/m, 0, 0;
     1, 0, 0, -d/m, 0, 0;
     0, 1, 0, 0, -d/m, 0;
     0, 0, 1, 0, 0, 0]';

B = [0, 0;
     0, 0;
     0, 0;
     1/m, 0;
     0, 1/m;
     -r/J, 0];

% K = [0.3162,0,-6.4109,1.1748,0,-1.7795;
%     0,1.4142,0,0,3.5098,0];
K = [0.3162,0,-6.4109,1.1748,0,-1.7795;
    0,1,0,0,2.9838,0];

Am = A-B*K;

Bm = B;

% P = [0.3764,0,-0.5627,0.2069,0,-0.0489;
%     0,5.1051,0,0,6.0104,0;
%     -0.5627,0,9.6064,-2.0459,0,1.0833;
%     0.2069,0,-2.0459,0.716,0,-0.1838;
%     0,6.0104,0,0,14.9167,0;
%     -0.0489,0,1.0833,-0.1838,0,0.3172];

P = [0.3764,0,-0.5627,0.2069,0,-0.0489;
     0,3.0838,0,0,4.25,0;
     -0.5627,0,9.6064,-2.0459,0,1.0833;
     0.2069,0,-2.0459,0.716,0,-0.1838;
     0,4.25,0,0,12.6813,0;
     -0.0489,0,1.0833,-0.1838,0,0.3172];

gamma_x = eye(6);
gamma_r = eye(2);

sdot = zeros(size(s));

x = s(1:6);
xm = s(7:12);
kx = reshape(s(13:24), [2, 6]);
kr = reshape(s(25:end), [2,2]);

xddot = xd(4:end);
xd = xd(1:6);
r = (Bm'*Bm)\Bm'*(xddot-Am*xd);

f = kx*x + kr*r + [0; tau0];
e = x - xm;

u = [norm(f); atan2(f(1),f(2))];

sdot(1:6) = odefun2(t, s, u);
sdot(7:12) = Am*xm + Bm*r;
kxdot = -gamma_x * x * e' * P * B;
kxdot = kxdot';
sdot(13:24) = kxdot(:);
krdot = -gamma_r * r * e' * P * B;
krdot = krdot';
sdot(25:end) = krdot(:);
end

function u = nonlinearControl(x,xd)

m = 4.25;
d = 0.1;
g = 9.8;
r = 0.26;
J = 0.0475;

tau0 = m*g;

A = [0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, -tau0/m, 0, 0;
     1, 0, 0, -d/m, 0, 0;
     0, 1, 0, 0, -d/m, 0;
     0, 0, 1, 0, 0, 0]';

B = [0, 0;
     0, 0;
     0, 0;
     1/m, 0;
     0, 1/m;
     -r/J, 0];
 
% K = [0.3162,0,-6.4109,1.1748,0,-1.7795;
%     0,1.4142,0,0,3.5098,0];
K = [0.3162,0,-6.4109,1.1748,0,-1.7795;
    0,1,0,0,2.9838,0];

xddot = xd(4:end);
xd = xd(1:6);

r = (B'*B)\B'*(xddot+(B*K-A)*xd);
f = r-K*x+[sin(x(3)); cos(x(3))]*tau0;
u = [norm(f); atan2(f(1),f(2))];
end

function sdot = odefun4(t, s, xd)

m = 4.25;
d = 0.1;
g = 9.8;
r = 0.26;
J = 0.0475;

tau0 = m*g;

x = s(1:6);
xm = s(7:12);
kx = reshape(s(13:24), [2, 6]);
kr = reshape(s(25:28), [2,2]);
alpha = reshape(s(29:end), [2,2]);

A = [0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, -tau0/m, 0, 0;
     1, 0, 0, -d/m, 0, 0;
     0, 1, 0, 0, -d/m, 0;
     0, 0, 1, 0, 0, 0]';

B = [0, 0;
     0, 0;
     0, 0;
     rot(x(3));
     -r/J, 0];
 
% K = [0.3162,0,-6.4109,1.1748,0,-1.7795;
%     0,1.4142,0,0,3.5098,0];
K = [0.3162,0,-6.4109,1.1748,0,-1.7795;
    0,1,0,0,2.9838,0];

Am = A-B*K;
Bm = B;

% P = [0.3764,0,-0.5627,0.2069,0,-0.0489;
%     0,5.1051,0,0,6.0104,0;
%     -0.5627,0,9.6064,-2.0459,0,1.0833;
%     0.2069,0,-2.0459,0.716,0,-0.1838;
%     0,6.0104,0,0,14.9167,0;
%     -0.0489,0,1.0833,-0.1838,0,0.3172];

P = [0.3764,0,-0.5627,0.2069,0,-0.0489;
     0,3.0838,0,0,4.25,0;
     -0.5627,0,9.6064,-2.0459,0,1.0833;
     0.2069,0,-2.0459,0.716,0,-0.1838;
     0,4.25,0,0,12.6813,0;
     -0.0489,0,1.0833,-0.1838,0,0.3172];

gamma_x = eye(6);
gamma_r = eye(2);
gamma_a = eye(2);

sdot = zeros(size(s));



xddot = xd(4:end);
xd = xd(1:6);

r = (B'*B)\B'*(xddot+(B*K-A)*xd);

Phi = [sin(x(3)); cos(x(3))];

f = kx*x + kr*r - alpha * Phi;
u = [norm(f); atan2(f(1),f(2))];
e = x - xm;
% e(4:5) = rot(x(3))'*e(4:5);

sdot(1:6) = odefun2(t, s, u);
sdot(7:12) = Am*xm + Bm*r;
kxdot = -gamma_x * x * e' * P * B;
kxdot = kxdot';
sdot(13:24) = kxdot(:);
krdot = -gamma_r * r * e' * P * B;
krdot = krdot';
sdot(25:28) = krdot(:);
alphadot = gamma_a * Phi * e' * P * B;
alphadot = alphadot';
sdot(29:end) = alphadot(:);
end

function sdot = odefun5(t, s, xd)

m = 4.25;
d = 0.1;
g = 9.8;
r = 0.26;
J = 0.0475;

tau0 = m*g;

x = s(1:6);
xm = s(7:12);
kx = reshape(s(13:24), [2, 6]);
kr = reshape(s(25:28), [2,2]);
alpha = reshape(s(29:end), [2,1]);

A = [0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, -tau0/m, 0, 0;
     1, 0, 0, -d/m, 0, 0;
     0, 1, 0, 0, -d/m, 0;
     0, 0, 1, 0, 0, 0]';

B = [0, 0;
     0, 0;
     0, 0;
     1/m, 0;
     0, 1/m;
     -r/J, 0];
 
% K = [0.3162,0,-6.4109,1.1748,0,-1.7795;
%     0,1.4142,0,0,3.5098,0];
K = [0.3162,0,-6.4109,1.1748,0,-1.7795;
    0,1,0,0,2.9838,0];

Am = A-B*K;
Bm = B;

% P = [0.3764,0,-0.5627,0.2069,0,-0.0489;
%     0,5.1051,0,0,6.0104,0;
%     -0.5627,0,9.6064,-2.0459,0,1.0833;
%     0.2069,0,-2.0459,0.716,0,-0.1838;
%     0,6.0104,0,0,14.9167,0;
%     -0.0489,0,1.0833,-0.1838,0,0.3172];

P = [0.3764,0,-0.5627,0.2069,0,-0.0489;
     0,3.0838,0,0,4.25,0;
     -0.5627,0,9.6064,-2.0459,0,1.0833;
     0.2069,0,-2.0459,0.716,0,-0.1838;
     0,4.25,0,0,12.6813,0;
     -0.0489,0,1.0833,-0.1838,0,0.3172];

gamma_x = eye(6);
gamma_r = eye(2);
gamma_a = 1;

sdot = zeros(size(s));

xddot = xd(4:end);
xd = xd(1:6);

r = (B'*B)\B'*(xddot+(B*K-A)*xd);

Phi = 1;

f = kx*x + kr*r - alpha * Phi;
u = [norm(f); atan2(f(1),f(2))];
e = x - xm;
% e(4:5) = rot(x(3))'*e(4:5);

sdot(1:6) = odefun2(t, s, u);
sdot(7:12) = Am*xm + Bm*r;
kxdot = -gamma_x * x * e' * P * B;
kxdot = kxdot';
sdot(13:24) = kxdot(:);
krdot = -gamma_r * r * e' * P * B;
krdot = krdot';
sdot(25:28) = krdot(:);
alphadot = gamma_a * Phi * e' * P * B;
alphadot = alphadot';
sdot(29:end) = alphadot(:);
end

% =======================================================================
% =========================== Helper Functions ==========================
% =======================================================================

function R = rot(theta)
R = [cos(theta), -sin(theta);
     sin(theta), cos(theta)];
end

function plotStates(t, x, r, save_path)
fig = figure('units', 'normalized', 'outerposition', [0,0,1,1]);
labels = {'$x$', '$y$', '$\theta$', '$\dot{x}$', '$\dot{y}$', '$\dot{\theta}$'};
for i = 1:6
    ax = subplot(3,2,i);
    plot(t, x(:,i),'linewidth',2);
    hold on
    plot(t, r(:,i), '--', 'linewidth',2);
    if i >= 5
        xlabel('Time [s]')
    end
    ylabel(labels{i}, 'Interpreter', 'latex')
    ax.FontSize = 18;
    if diff(ax.YLim) < .01
        ylim([-.01, .01]+mean(ax.YLim))
    end
    grid on
end

saveas(fig, "Figures/PNG/"+save_path, 'png')
end

function plotAdaptiveStates(t, x, r, title_suff)
fig = figure('units', 'normalized', 'outerposition', [0,0,1,1]);
labels = {'$x$', '$y$', '$\theta$', '$\dot{x}$', '$\dot{y}$', '$\dot{\theta}$'};
for i = 1:6
    ax = subplot(3,2,i);
    plot(t, x(:,i), 'linewidth', 2);
    hold on
    plot(t, x(:,i+6), '--', 'linewidth', 2);
    plot(t, r(:,i), ':', 'linewidth', 2);
    if i >= 5
        xlabel('Time [s]')
    end
    ylabel(labels{i}, 'Interpreter', 'latex')
    ax.FontSize = 18;
    if diff(ax.YLim) < .01
        ylim([-.01, .01]+mean(ax.YLim))
    end
    grid on
end
sgtitle("Adaptive Control" + title_suff);
legend('Actual', 'Model', 'Reference')

mask = ~isspace(title_suff);
c = char(title_suff);
saveas(fig, "Figures/PNG/AdaptiveControl" + c(mask), 'png')

fig = figure;
plot(t, x(:,13:24), 'linewidth', 2)
ylabel('k_x')
xlabel('Time [s]')
title("Gains")

saveas(fig, "Figures/PNG/AdaptiveControl" + c(mask) + "Kx", 'png')

fig = figure;
plot(t, x(:,25:end), 'linewidth', 2)
ylabel('k_r')
xlabel('Time [s]')
title("Gains")

saveas(fig, "Figures/PNG/AdaptiveControl" + c(mask) + "Kr", 'png')
end

function plotAdaptiveStates2(t, x, r, subplot_title)
fig = figure('units', 'normalized', 'outerposition', [0,0,1,1]);
labels = {'$x$', '$y$', '$\theta$', '$\dot{x}$', '$\dot{y}$', '$\dot{\theta}$'};
for i = 1:6
    ax = subplot(3,2,i);
    plot(t, x(:,i), 'linewidth', 2);
    hold on
    plot(t, x(:,i+6), '--', 'linewidth', 2);
    plot(t, r(:,i), ':', 'linewidth', 2);
    if i >= 5
        xlabel('Time [s]')
    end
    ylabel(labels{i}, 'Interpreter', 'latex')
    ax.FontSize = 18;
    if diff(ax.YLim) < .01
        ylim([-.01, .01]+mean(ax.YLim))
    end
    grid on
end
sgtitle(subplot_title);
legend('Actual', 'Model', 'Reference')

mask = ~isspace(subplot_title);
c = char(subplot_title);
saveas(fig, "Figures/PNG/" + c(mask), 'png')

fig = figure;
plot(t, x(:,13:24), 'linewidth', 2)
ylabel('k_x')
xlabel('Time [s]')
title("Gains")

saveas(fig, "Figures/PNG/" + c(mask) + "Kx", 'png')

fig = figure;
plot(t, x(:,25:28), 'linewidth', 2)
ylabel('k_r')
xlabel('Time [s]')
title("Gains")

saveas(fig, "Figures/PNG/" + c(mask) + "Kr", 'png')

fig = figure;
plot(t, x(:,29:end), 'linewidth', 2)
ylabel('\alpha')
xlabel('Time [s]')
title("Gains")

saveas(fig, "Figures/PNG/" + c(mask) + "Ka", 'png')
end