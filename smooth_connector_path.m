function [time, position, velocity, acceleration, kineticEnergy] = smooth_connector_path(period)
linkLength = 0.2; connectorLength = 0.22; triangleSide = 0.08;
platformCenter = [0.134; 0.547];
frameCount = 360; connectorDirectionCount = 720;
dt = period/frameCount; tolerance = 1e-10;
anchor2 = [(0.5 + cos(pi/3) + 2*cos(pi/6) + cos(5*pi/6 - 2*pi/5))*linkLength; ...
    (sin(2*pi/5) + sin(4*pi/5) + 1 + sin(pi/3) + sin(2*pi/3) + ...
    sin(2*pi/3 + 2*pi/5))*linkLength];
anchor3 = [-(0.5 + cos(pi/3) + cos(pi/6) + ...
    cos(4*pi/5 - pi/3 + cos(2*pi/5 - pi/3)))*linkLength; ...
    anchor2(2) + sin(2*pi/3)*linkLength];
leftAnchors = [zeros(2,1),anchor2,anchor3];
baseAngles = [0,2*pi/3,-2*pi/3];
targetMotorAngles = deg2rad([108.237419223938,70.9415300598879; ...
    207.797310496314,198.958829937579;-4.12950605013165,-41.2198863746436]);
initialMotorAngles = targetMotorAngles - baseAngles(:);
vertexPhases = deg2rad([-150,-30,90]);
vertexRadius = triangleSide/sqrt(3);
rotationAngles = (0:frameCount)'*(2*pi/frameCount);
platformSideAngles = [pi/6,pi/6,5*pi/6];
candidateAngles = -pi + (0:connectorDirectionCount-1)*(2*pi/connectorDirectionCount);
neighbors = [2 3;3 1;1 2];
connectorAngles = zeros(frameCount+1,3);
previousStates = cell(1,3);
kineticEnergy = zeros(frameCount+1,1);

for frame = 1:frameCount+1
    alpha = rotationAngles(frame);
    vertices = zeros(2,3);
    for v = 1:3
        phase = vertexPhases(v)+alpha;
        vertices(:,v) = platformCenter+vertexRadius*[cos(phase);sin(phase)];
    end
    for leg = 1:3
        neighborVectors = vertices(:,neighbors(leg,:))-vertices(:,leg);
        state = chooseState(vertices(:,leg),leg,neighborVectors,previousStates{leg}, ...
            frame==1,leftAnchors,baseAngles,linkLength,connectorLength, ...
            candidateAngles,deg2rad(2),tolerance,initialMotorAngles,2*pi/frameCount,dt);
        connectorAngles(frame,leg) = state.connectorAngle;
        kineticEnergy(frame) = kineticEnergy(frame)+state.kineticEnergy;
        previousStates{leg} = state;
    end
end

time = (0:frameCount)'*dt;
position = zeros(frameCount+1,3);
for leg = 1:3
    phi = unwrap(connectorAngles(:,leg));
    position(:,leg) = rotationAngles + platformSideAngles(leg) - phi - pi;
end
velocity = zeros(size(position)); acceleration = zeros(size(position));
for leg = 1:3
    velocity(:,leg) = gradient(position(:,leg),dt);
    acceleration(:,leg) = gradient(velocity(:,leg),dt);
end
end

function state = chooseState(vertex,leg,neighborVectors,previousState,firstFrame, ...
    leftAnchors,baseAngles,linkLength,connectorLength,candidateAngles,margin, ...
    tolerance,initialMotorAngles,alphaStep,dt)
best = inf; state = [];
for phi = candidateAngles
    endpoint = vertex+connectorLength*[cos(phi);sin(phi)];
    connectorVector = vertex-endpoint;
    if min(arrayfun(@(k) alignmentMargin(connectorVector,neighborVectors(:,k)),1:2)) < margin
        continue
    end
    R = rotationMatrix(-baseAngles(leg));
    localEndpoint = R*(endpoint-leftAnchors(:,leg));
    left = circleIntersections([0;0],linkLength,localEndpoint,linkLength,tolerance);
    right = circleIntersections([linkLength;0],linkLength,localEndpoint,linkLength,tolerance);
    for i = 1:size(left,2)
        for j = 1:size(right,2)
            eL = left(:,i); eR = right(:,j);
            margins = [alignmentMargin(-eL,localEndpoint-eL), ...
                alignmentMargin([linkLength;0]-eR,localEndpoint-eR), ...
                alignmentMargin(eL-localEndpoint,eR-localEndpoint), ...
                alignmentMargin(eL-localEndpoint,R*connectorVector), ...
                alignmentMargin(eR-localEndpoint,R*connectorVector)];
            if min(margins) < margin, continue, end
            assembly = cross2d(eR-eL,localEndpoint-eL);
            if abs(assembly)<=tolerance, continue, end
            motors = [atan2(eL(2),eL(1)),atan2(eR(2),eR(1)-linkLength)];
            baseLeft = leftAnchors(:,leg);
            baseRight = baseLeft+rotationMatrix(baseAngles(leg))*[linkLength;0];
            worldLeft = baseLeft+rotationMatrix(baseAngles(leg))*eL;
            worldRight = baseLeft+rotationMatrix(baseAngles(leg))*eR;
            centers = [(baseLeft+worldLeft)/2, (worldLeft+endpoint)/2, ...
                (endpoint+worldRight)/2, (worldRight+baseRight)/2, ...
                (endpoint+vertex)/2];
            orientations = [atan2(worldLeft(2)-baseLeft(2),worldLeft(1)-baseLeft(1)), ...
                atan2(endpoint(2)-worldLeft(2),endpoint(1)-worldLeft(1)), ...
                atan2(worldRight(2)-endpoint(2),worldRight(1)-endpoint(1)), ...
                atan2(baseRight(2)-worldRight(2),baseRight(1)-worldRight(1)), ...
                atan2(vertex(2)-endpoint(2),vertex(1)-endpoint(1))];
            if firstFrame
                d = wrapAngle(motors-initialMotorAngles(leg,:));
                score = sum(d.^2);
                kinetic = 0;
                qRate = 0;
                centerVelocity = zeros(size(centers));
                angularVelocity = zeros(size(orientations));
            else
                centerVelocity = (centers-previousState.centers)/dt;
                angularVelocity = wrapAngle(orientations-previousState.orientations)/dt;
                inertia = [(linkLength^2+0.1^2)/12*ones(1,4), ...
                    (connectorLength^2+0.1^2)/12];
                kinetic = 0.5*(sum(centerVelocity(:).^2)+sum(inertia.*angularVelocity.^2));
                dPhi = wrapAngle(phi-previousState.connectorAngle);
                d = wrapAngle(motors-previousState.motorAngles);
                qRate = alphaStep-dPhi;
                qAccel = qRate-previousState.qRate;
                flip = 100*(sign(assembly)~=previousState.assemblySign);
                score = 0.1*sum(d.^2)+25*qRate^2+5*qAccel^2+flip;
            end
            if score<best
                best=score; state.motorAngles=motors; state.connectorAngle=phi;
                state.assemblySign=sign(assembly);
                state.qRate=qRate; state.centers=centers;
                state.orientations=orientations; state.kineticEnergy=kinetic;
            end
        end
    end
end
if isempty(state), error('No feasible state for leg %d.',leg); end
end

function x = circleIntersections(c0,r0,c1,r1,tol)
d=c1-c0; L=norm(d); x=zeros(2,0);
if L<=tol||L>r0+r1+tol||L<abs(r0-r1)-tol, return, end
a=(r0^2-r1^2+L^2)/(2*L); h2=r0^2-a^2; if h2<=tol, return, end
u=d/L; p=[-u(2);u(1)]; f=c0+a*u; h=sqrt(h2); x=[f+h*p,f-h*p];
end
function R=rotationMatrix(a), R=[cos(a),-sin(a);sin(a),cos(a)]; end
function z=cross2d(a,b), z=a(1)*b(2)-a(2)*b(1); end
function m=alignmentMargin(a,b), x=atan2(abs(cross2d(a,b)),dot(a,b)); m=min(x,pi-x); end
function a=wrapAngle(a), a=mod(a+pi,2*pi)-pi; end
