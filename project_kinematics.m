AB = 0.2;
BC = 0.2;
CD = 0.2;
DE = 0.2;
EA = 0.2;
CF = 0.2;
FF = 0.2;
linkLengths = [AB, BC, CD, DE, EA, CF, FF];
A1A2x = (0.5+cos(pi/3)+cos(pi/6)*2+cos(5*pi/6-2*pi/5))*0.2;
A1A2y = (sin(2*pi/5)+sin(4*pi/5)+1+sin(pi/3)+sin(2*pi/3)+sin(2*pi/3+2*pi/5))*0.2;
A1A2t = 2*pi/3;
A1A3x = -(0.5+cos(pi/3)+cos(pi/6)+cos(4*pi/5-pi/3+cos(2*pi/5-pi/3)))*0.2;
A1A3y = A1A2y+sin(2*pi/3)*0.2;
A1A3t = -2*pi/3;

% fo1rward kinematics
o1 = pi*3/5;
o7 = pi*3/5+A1A2t;
x_2b = A1A2x+AB*cos(o7);
y_2b = A1A2y+AB*sin(o7);
o13 = pi*3/5+A1A3t;
x_3b = A1A3x+AB*cos(o13);
y_3b = A1A3y+AB*sin(o13);
f = @(o, o1, o7, o13) [AB*cos(o1) + BC*cos(o(3)+o1) - EA - DE*cos(o(2)) - CD*cos(o(4)+o(2));
    AB*sin(o1) + BC*sin(o(3)+o1) - DE*sin(o(2)) - CD*sin(o(4)+o(2));
    x_2b + BC*cos(o(9)+o7) - A1A2x-EA*cos(A1A2t) - DE*cos(o(8)) - CD*cos(o(10)+o(8));
    y_2b + BC*sin(o(9)+o7) - A1A2y-EA*sin(A1A2t) - DE*sin(o(8)) - CD*sin(o(10)+o(8));
    x_3b + BC*cos(o(15)+o13) - A1A3x-EA*cos(A1A3t) - DE*cos(o(14)) - CD*cos(o(16)+o(14));
    y_3b + BC*sin(o(15)+o13) - A1A3y-EA*sin(A1A3t) - DE*sin(o(14)) - CD*sin(o(16)+o(14));
    AB*cos(o1) + BC*cos(o(3)+o1) + CF*cos(o(3)+o1+o(5)) - FF*cos(o(3)+o1+o(5)+o(6)) - ...
    (x_2b + BC*cos(o(9)+o7) + CF*cos(o(9)+o7+o(11)));
    x_2b + BC*cos(o(9)+o7) + CF*cos(o(9)+o7+o(11)) + FF*cos(o(9)+o7+o(11)+o(12)) - ...
    (x_3b + BC*cos(o(15)+o13) + CF*cos(o(15)+o13+o(17)));
    x_3b + BC*cos(o(15)+o13) + CF*cos(o(15)+o13+o(17)) + FF*cos(o(18)) - ...  %o(15)+o13+o(17)+
    (AB*cos(o1) + BC*cos(o(3)+o1) + CF*cos(o(3)+o1+o(5)));
    AB*sin(o1) + BC*sin(o(3)+o1) + CF*sin(o(3)+o1+o(5)) + FF*sin(o(3)+o1+o(5)+o(6)) - ...
    (y_2b + BC*sin(o(9)+o7) + CF*sin(o(9)+o7+o(11)));
    y_2b + BC*sin(o(9)+o7) + CF*sin(o(9)+o7+o(11)) + FF*sin(o(9)+o7+o(11)+o(12)) - ...
    (y_3b + BC*sin(o(15)+o13) + CF*sin(o(15)+o13+o(17)));
    y_3b + BC*sin(o(15)+o13) + CF*sin(o(15)+o13+o(17)) + FF*sin(o(18)) - ... %o(15)+o13+o(17)+
    (AB*sin(o1) + BC*sin(o(3)+o1) + CF*sin(o(3)+o1+o(5)))];
oSol = fsolve(@(o) f(o, o1, o7, o13), [pi*3/5;pi*2/5;-pi*2/5;pi*2/5;pi*2/5;0; ... %[output:group:57ff3a3e] %[output:8bee838e] %[output:659063a9]
    pi*3/5;pi*2/5;-pi*2/5;pi*2/5;pi*2/5;0; ... %[output:8bee838e] %[output:659063a9]
    pi*3/5;pi*2/5;-pi*2/5;pi*2/5;pi*2/5;0]) %[output:group:57ff3a3e] %[output:8bee838e] %[output:659063a9]

% --- Plot solved configuration ---

function [A1, B1, C1, D1, E1, F1, A2, B2, C2, D2, E2, F2, A3, B3, C3, D3, E3, F3] = calc(AB, BC, CD, DE, EA, CF, A1A2x, A1A2y, A1A2t, A1A3x, A1A3y, A1A3t, oSol)
% Constants
x_2b = A1A2x+AB*cos(oSol(7));
y_2b = A1A2y+AB*sin(oSol(7));
x_3b = A1A3x+AB*cos(oSol(13));
y_3b = A1A3y+AB*sin(oSol(13));
% Fixed anchor points
A1 = [0; 0];
A2 = [A1A2x; A1A2y];
A3 = [A1A3x; A1A3y];
E1 = [EA; 0];
E2 = [EA*cos(A1A2t)+A1A2x;EA*sin(A1A2t)+A1A2y];
E3 = [EA*cos(A1A3t)+A1A3x;EA*sin(A1A3t)+A1A3y];

% Base joints
B1 = [AB*cos(oSol(1)); AB*sin(oSol(1))];
B2 = [x_2b; y_2b];
B3 = [x_3b; y_3b];

% Branch 1
C1 = B1 + BC * [cos(oSol(3) + oSol(1)); sin(oSol(3) + oSol(1))];
F1 = C1 + CF * [cos(oSol(3) + oSol(1) + oSol(5)); sin(oSol(3) + oSol(1) + oSol(5))];
D1 = E1  + DE * [cos(oSol(2)); sin(oSol(2))];

% Branch 2
C2 = B2 + BC * [cos(oSol(9) + oSol(7)); sin(oSol(9) + oSol(7))];
F2 = C2 + CF * [cos(oSol(9) + oSol(7) + oSol(11)); sin(oSol(9) + oSol(7) + oSol(11))];
D2 = E2  + DE * [cos(oSol(8)); sin(oSol(8))];

% Branch 3
C3 = B3 + BC * [cos(oSol(15) + oSol(13)); sin(oSol(15) + oSol(13))];
F3 = C3 + CF * [cos(oSol(15) + oSol(13) + oSol(17)); sin(oSol(15) + oSol(13) + oSol(17))];
D3 = E3 + DE * [cos(oSol(14)); sin(oSol(14))];
end

function draw(A1, B1, C1, D1, E1, F1, A2, B2, C2, D2, E2, F2, A3, B3, C3, D3, E3, F3)
title('Parallel 18-Variable Robot')
% Plot branches
plot([A1(1) B1(1) C1(1) F1(1) C1(1) D1(1) E1(1)], [A1(2) B1(2) C1(2) F1(2) C1(2) D1(2) E1(2)], 'o-', 'LineWidth', 2)
plot([A2(1) B2(1) C2(1) F2(1) C2(1) D2(1) E2(1)], [A2(2) B2(2) C2(2) F2(2) C2(2) D2(2) E2(2)], 'o-', 'LineWidth', 2)
plot([A3(1) B3(1) C3(1) F3(1) C3(1) D3(1) E3(1)], [A3(2) B3(2) C3(2) F3(2) C3(2) D3(2) E3(2)], 'o-', 'LineWidth', 2)

% Plot moving triangle between the three end points
plot([F1(1) F2(1) F3(1) F1(1)], [F1(2) F2(2) F3(2) F1(2)], 'k-', 'LineWidth', 2)

% Plot fixed points
plot(A1(1), A1(2), 'ks', 'MarkerFaceColor', 'k')
plot(A2(1), A2(2), 'ks', 'MarkerFaceColor', 'k')
plot(A3(1), A3(2), 'ks', 'MarkerFaceColor', 'k')
plot(E1(1), E1(2), 'ks', 'MarkerFaceColor', 'k')
plot(E2(1), E2(2), 'ks', 'MarkerFaceColor', 'k')
plot(E3(1), E3(2), 'ks', 'MarkerFaceColor', 'k')

legend('Branch 1', 'Branch 2', 'Branch 3', 'End-effector triangle', 'Fixed points', 'Location', 'best')
xlabel('x')
ylabel('y')
hold off
end
figure %[output:8859ef7d]
hold on %[output:8859ef7d]
axis equal %[output:8859ef7d]
grid on %[output:8859ef7d]
oSol(1) = o1;
oSol(7) = o7;
oSol(13) = o13;
[A1, B1, C1, D1, E1, F1, A2, B2, C2, D2, E2, F2, A3, B3, C3, D3, E3, F3] = calc(AB, BC, CD, DE, EA, CF, A1A2x, A1A2y, A1A2t, A1A3x, A1A3y, A1A3t, oSol);
draw(A1, B1, C1, D1, E1, F1, A2, B2, C2, D2, E2, F2, A3, B3, C3, D3, E3, F3); %[output:8859ef7d]

options = optimoptions("fsolve", Display="none");

nFrames = 50;
dTheta = 4*pi/nFrames;

x0 = [pi*3/5; pi*2/5; -pi*2/5; pi*2/5; pi*2/5; 0; ...
    pi*3/5; pi*2/5; -pi*2/5; pi*2/5; pi*2/5; 0; ...
    pi*3/5; pi*2/5; -pi*2/5; pi*2/5; pi*2/5; 0];
warning('off','all')
figure %[output:4d4af3fa]
T = nFrames/5;
do = (pi/2-2*pi/5)*(2/nFrames);
for k = 1:nFrames %[output:group:962a39cf]
    %oSol = lsqnonlin(@(o) res(o, theta18+dTheta*k), o0, [], [], options);
    x0(18) = x0(18) + dTheta;   % seed the motion
    if (mod(k,T) == 0)
        do = -do;
    end
    o1 = o1 - do;
    o7 = o7 - do;
    o13 = o13 - do;
    [oSol,~,exitflag] = fsolve(@(o) f(o, o1, o7, o13), x0, options);
    if exitflag <= 0
        warning('fsolve failed at frame %d', k)
        break
    end
    x0 = oSol;
    clf %[output:4d4af3fa]
    hold on
    axis equal
    grid on
    oSol(1) = o1;
    oSol(7) = o7;
    oSol(13) = o13;
    [A1, B1, C1, D1, E1, F1, A2, B2, C2, D2, E2, F2, A3, B3, C3, D3, E3, F3] = calc(AB, BC, CD, DE, EA, CF, A1A2x, A1A2y, A1A2t, A1A3x, A1A3y, A1A3t, oSol);
    draw(A1, B1, C1, D1, E1, F1, A2, B2, C2, D2, E2, F2, A3, B3, C3, D3, E3, F3);
    % put your existing plotting code here, using this frame's oSol

    drawnow
    %pause(0.03)
end %[output:group:962a39cf]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"inline"}
%---
%[output:8bee838e]
%   data: {"dataType":"text","outputData":{"text":"\n<a href = \"matlab: helpview('optim','eqn_solved','CSHelpWindow');\">Equation solved<\/a>.\n\nfsolve completed because the vector of function values is near zero\nas measured by the value of the <a href = \"matlab: helpview('optim','fcn_tolerance_fsolve','CSHelpWindow');\">function tolerance<\/a>, and\nthe <a href = \"matlab: helpview('optim','appears_regular','CSHelpWindow');\">problem appears regular<\/a> as measured by the gradient.\n\n<<a href = \"matlab: createExitMsg({'optim:fsolve:Exit1basicLM','fsolve'},{'optim:fsolve:Exit1detailedLM','2.815894e-14','1.000000e-10','1.398412e-25','1.000000e-03'},true,true);;\">stopping criteria details<\/a>>\n","truncated":false}}
%---
%[output:659063a9]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"oSol","rows":18,"type":"double","value":[["1.8850"],["1.7217"],["-0.9000"],["0.4998"],["-0.0279"],["1.1493"],["1.8850"],["-1.0834"],["-2.0626"],["4.0081"],["2.1502"],["-0.9373"],["1.8850"],["-1.2499"],["-1.8596"]]}}
%---
%[output:8859ef7d]
%   data: {"dataType":"image","outputData":{"height":337,"width":560}}
%---
%[output:4d4af3fa]
%   data: {"dataType":"image","outputData":{"height":337,"width":560}}
%---
