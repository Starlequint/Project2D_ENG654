% Build and run a Simscape Multibody replay of the notebook's full-turn path.
% The original model is left untouched; a separate *_six_input_rotation.slx is
% created next to this script. Run this script in MATLAB to replay the motion.

projectDir = fileparts(mfilename('fullpath'));
sourceModel = 'Kinematically_Redundant_Planar_Parallel_Manipulator';
animationModel = [sourceModel '_six_input_rotation'];
sourceFile = fullfile(projectDir, [sourceModel '.slx']);
animationFile = fullfile(projectDir, [animationModel '.slx']);

if ~isfile(sourceFile)
    error('Source Simscape model not found: %s', sourceFile);
end
if bdIsLoaded(animationModel) && ~isfile(animationFile)
    error('Close the unsaved animation model before running this script again.');
end
isExistingAnimation = isfile(animationFile);

AB = 0.2;
BC = 0.2;
CD = 0.2;
DE = 0.2;
EA = 0.2;
CF = 0.2;
FF = 0.2;
A1A2t = 2*pi/3;
A1A2x = (0.5 + cos(pi/3) + 2*cos(pi/6) + cos(5*pi/6 - 2*pi/5))*AB;
A1A2y = (sin(2*pi/5) + sin(4*pi/5) + 1 + sin(pi/3) + ...
    sin(2*pi/3) + sin(2*pi/3 + 2*pi/5))*AB;
A1A5x = -(0.5 + cos(pi/3) + cos(pi/6) + ...
    cos(4*pi/5 - pi/3 + cos(2*pi/5 - pi/3)))*AB;
A1A5y = A1A2y + sin(2*pi/3)*AB;
A1A3x = A1A2x;
A1A3y = A1A2y;
A1A3t = A1A2t;
A1A5t = -2*pi/3;
OA = 0;
EO = EA;

assignin('base', 'AB', AB);
assignin('base', 'BC', BC);
assignin('base', 'CD', CD);
assignin('base', 'DE', DE);
assignin('base', 'EA', EA);
assignin('base', 'CF', CF);
assignin('base', 'FF', FF);
assignin('base', 'A1A2t', A1A2t);
assignin('base', 'A1A2x', A1A2x);
assignin('base', 'A1A2y', A1A2y);
assignin('base', 'A1A3x', A1A3x);
assignin('base', 'A1A3y', A1A3y);
assignin('base', 'A1A3t', A1A3t);
assignin('base', 'A1A5x', A1A5x);
assignin('base', 'A1A5y', A1A5y);
assignin('base', 'A1A5t', A1A5t);
assignin('base', 'OA', OA);
assignin('base', 'EO', EO);

[time, position, velocity, acceleration] = notebookMotorTrajectory();

for motorIndex = 1:6
    signalNames = {'position', 'velocity', 'acceleration'};
    signalValues = {position, velocity, acceleration};
    for signalIndex = 1:numel(signalNames)
        variableName = sprintf('robotMotor%d_%s', motorIndex, ...
            signalNames{signalIndex});
        assignin('base', variableName, ...
            timeseries(signalValues{signalIndex}(:, motorIndex), time));
    end
end

if isExistingAnimation
    load_system(animationFile);
    for motorIndex = 1:6
        for signalName = {'position', 'velocity', 'acceleration'}
            blockName = sprintf('NotebookMotor%d_%s', motorIndex, ...
                signalName{1});
            if isempty(find_system(animationModel, 'SearchDepth', 1, ...
                    'Name', blockName))
                error('Existing model is missing the expected block %s.', ...
                    blockName);
            end
        end
    end
else
    new_system(animationModel, 'FromFile', sourceFile);
    load_system(animationModel);
end
set_param(animationModel, 'StopTime', num2str(time(end), '%.15g'));

if ~isExistingAnimation
% Match the notebook's 0.23 m connector and 0.10 m platform sides.
for solidName = {'EF1', 'EF2', 'EF3'}
    set_param([animationModel '/' solidName{1}], ...
        'BrickDimensions', '[0.23 0.1 0.1]');
end
for solidName = {'F12', 'F23', 'F31'}
    set_param([animationModel '/' solidName{1}], ...
        'BrickDimensions', '[0.10 0.1 0.1]');
end

motorConverters = {'Simulink-PS Converter', ...
    'Simulink-PS Converter1', 'Simulink-PS Converter2', ...
    'Simulink-PS Converter3', 'Simulink-PS Converter4', ...
    'Simulink-PS Converter5'};
motorJoints = {'Revolute Joint', 'Revolute Joint3', ...
    'Revolute Joint5', 'Revolute Joint8', ...
    'Revolute Joint10', 'Revolute Joint13'};

for motorIndex = 1:6
    converterPath = [animationModel '/' motorConverters{motorIndex}];
    set_param(converterPath, 'FilteringAndDerivatives', 'provide');
    set_param(converterPath, 'UdotUserProvided', '2');
    portHandles = get_param(converterPath, 'PortHandles');
    if numel(portHandles.Inport) ~= 3
        error('Expected position, velocity, and acceleration inputs at %s.', ...
            converterPath);
    end

    jointPath = [animationModel '/' motorJoints{motorIndex}];
    if motorIndex == 2 || motorIndex == 4 || motorIndex == 6
        set_param(jointPath, ...
            'MotionActuationMode', 'InputMotion', ...
            'TorqueActuationMode', 'ComputedTorque');
        converterPorts = get_param(converterPath, 'PortHandles');
        jointPorts = get_param(jointPath, 'PortHandles');
        add_line(animationModel, converterPorts.RConn(1), jointPorts.LConn(2), ...
            'autorouting', 'on');
    end
    set_param(jointPath, ...
        'PositionTargetValue', num2str(rad2deg(position(1, motorIndex)), '%.15g'), ...
        'PositionTargetPriority', 'High', ...
        'TorqueActuationMode', 'ComputedTorque');

    portHandles = get_param(converterPath, 'PortHandles');
    inputBlocks = {sprintf('NotebookMotor%d_position', motorIndex), ...
        sprintf('NotebookMotor%d_velocity', motorIndex), ...
        sprintf('NotebookMotor%d_acceleration', motorIndex)};
    signalNames = {'position', 'velocity', 'acceleration'};
    xPosition = 900 + (motorIndex - 1)*180;
    for signalIndex = 1:3
        inputPath = [animationModel '/' inputBlocks{signalIndex}];
        add_block('simulink/Sources/From Workspace', inputPath, ...
            'VariableName', sprintf('robotMotor%d_%s', motorIndex, ...
            signalNames{signalIndex}), ...
            'Interpolate', 'on', ...
            'Position', [xPosition 40 + (signalIndex - 1)*55 ...
            xPosition + 140 60 + (signalIndex - 1)*55]);
        inputSource = get_param(inputPath, 'PortHandles');
        inputPort = portHandles.Inport(signalIndex);
        oldLine = get_param(inputPort, 'Line');
        if oldLine ~= -1
            oldSource = get_param(oldLine, 'SrcPortHandle');
            delete_line(animationModel, oldSource, inputPort);
        end
        add_line(animationModel, inputSource.Outport(1), inputPort, ...
            'autorouting', 'on');
    end
end

save_system(animationModel, animationFile);
end

if usejava('desktop')
    open_system(animationModel);
end
sim(animationModel);

fprintf('Simscape animation completed: %s\n', animationFile);

function [time, position, velocity, acceleration] = notebookMotorTrajectory()
linkLength = 0.2;
connectorLength = 0.23;
triangleSide = 0.1;
platformCenter = [0.134; 0.547];
frameCount = 360;
connectorDirectionCount = 360;
frameInterval = 5.76/frameCount;
tolerance = 1e-10;

anchor2 = [(0.5 + cos(pi/3) + 2*cos(pi/6) + ...
    cos(5*pi/6 - 2*pi/5))*linkLength; ...
    (sin(2*pi/5) + sin(4*pi/5) + 1 + sin(pi/3) + ...
    sin(2*pi/3) + sin(2*pi/3 + 2*pi/5))*linkLength];
anchor3 = [-(0.5 + cos(pi/3) + cos(pi/6) + ...
    cos(4*pi/5 - pi/3 + cos(2*pi/5 - pi/3)))*linkLength; ...
    anchor2(2) + sin(2*pi/3)*linkLength];
leftAnchors = [zeros(2, 1), anchor2, anchor3];
baseAngles = [0, 2*pi/3, -2*pi/3];
vertexPhases = deg2rad([-150, -30, 90]);
vertexRadius = triangleSide/sqrt(3);
rotationAngles = (0:frameCount - 1)*(2*pi/frameCount);
connectorAngles = -pi + (0:connectorDirectionCount - 1)* ...
    (2*pi/connectorDirectionCount);
allMotorAngles = zeros(frameCount, 3, 2);
previousStates = cell(1, 3);

for frameIndex = 1:frameCount
    alpha = rotationAngles(frameIndex);
    vertices = zeros(2, 3);
    for vertexIndex = 1:3
        phase = vertexPhases(vertexIndex) + alpha;
        vertices(:, vertexIndex) = platformCenter + ...
            vertexRadius*[cos(phase); sin(phase)];
    end

    for legIndex = 1:3
        state = chooseLegState(vertices(:, legIndex), legIndex, ...
            previousStates{legIndex}, frameIndex == 1, leftAnchors, ...
            baseAngles, linkLength, connectorLength, connectorAngles, tolerance);
        allMotorAngles(frameIndex, legIndex, :) = ...
            reshape(state.motorAngles, 1, 1, 2);
        previousStates{legIndex} = state;

        lengths = [norm(state.elbowLeft), ...
            norm(state.endpointLocal - state.elbowLeft), ...
            norm(state.endpointLocal - state.elbowRight), ...
            norm(state.elbowRight - [linkLength; 0]), ...
            norm(state.endpointGlobal - vertices(:, legIndex))];
        expectedLengths = [linkLength, linkLength, linkLength, ...
            linkLength, connectorLength];
        if any(abs(lengths - expectedLengths) > 1e-8)
            error('Notebook link-length check failed at frame %d, leg %d.', ...
                frameIndex, legIndex);
        end
    end

    sideLengths = [norm(vertices(:, 1) - vertices(:, 2)), ...
        norm(vertices(:, 2) - vertices(:, 3)), ...
        norm(vertices(:, 3) - vertices(:, 1))];
    if any(abs(sideLengths - triangleSide) > 1e-8)
        error('Notebook platform check failed at frame %d.', frameIndex);
    end
end

position = zeros(frameCount + 1, 6);
for legIndex = 1:3
    for motorSide = 1:2
        motorIndex = 2*(legIndex - 1) + motorSide;
        motorAngle = unwrap(allMotorAngles(:, legIndex, motorSide) + ...
            baseAngles(legIndex));
        closureAngle = motorAngle(1) + 2*pi*round( ...
            (motorAngle(end) - motorAngle(1))/(2*pi));
        position(:, motorIndex) = [motorAngle; closureAngle];
    end
end
time = (0:frameCount)'*frameInterval;
velocity = zeros(size(position));
acceleration = zeros(size(position));
for motorIndex = 1:size(position, 2)
    velocity(:, motorIndex) = gradient(position(:, motorIndex), frameInterval);
    acceleration(:, motorIndex) = ...
        gradient(velocity(:, motorIndex), frameInterval);
end
end

function state = chooseLegState(vertex, legIndex, previousState, firstFrame, ...
    leftAnchors, baseAngles, linkLength, connectorLength, connectorAngles, tolerance)
bestScore = inf;
state = [];

for connectorIndex = 1:numel(connectorAngles)
    connectorAngle = connectorAngles(connectorIndex);
    endpointGlobal = vertex + connectorLength* ...
        [cos(connectorAngle); sin(connectorAngle)];
    rotation = rotationMatrix(-baseAngles(legIndex));
    endpointLocal = rotation*(endpointGlobal - leftAnchors(:, legIndex));

    leftElbows = circleIntersections([0; 0], linkLength, ...
        endpointLocal, linkLength, tolerance);
    rightElbows = circleIntersections([linkLength; 0], linkLength, ...
        endpointLocal, linkLength, tolerance);

    for leftIndex = 1:size(leftElbows, 2)
        for rightIndex = 1:size(rightElbows, 2)
            elbowLeft = leftElbows(:, leftIndex);
            elbowRight = rightElbows(:, rightIndex);
            assemblySign = cross2d(elbowRight - elbowLeft, ...
                endpointLocal - elbowLeft);
            if abs(assemblySign) <= tolerance
                continue
            end

            motorAngles = [atan2(elbowLeft(2), elbowLeft(1)), ...
                atan2(elbowRight(2), elbowRight(1) - linkLength)];
            if firstFrame
                motorDelta = wrapAngle(motorAngles - deg2rad([90, 90]));
                score = sum(motorDelta.^2);
            else
                motorDelta = wrapAngle(motorAngles - previousState.motorAngles);
                connectorDelta = wrapAngle(connectorAngle - ...
                    previousState.connectorAngle);
                assemblyPenalty = 10* ...
                    (sign(assemblySign) ~= previousState.assemblySign);
                score = sum(motorDelta.^2) + ...
                    0.2*connectorDelta^2 + assemblyPenalty;
            end

            if score < bestScore
                bestScore = score;
                state.motorAngles = motorAngles;
                state.connectorAngle = connectorAngle;
                state.assemblySign = sign(assemblySign);
                state.elbowLeft = elbowLeft;
                state.elbowRight = elbowRight;
                state.endpointLocal = endpointLocal;
                state.endpointGlobal = endpointGlobal;
            end
        end
    end
end

if isempty(state)
    error('No reachable configuration for leg %d.', legIndex);
end
end

function intersections = circleIntersections(center0, radius0, center1, ...
    radius1, tolerance)
delta = center1 - center0;
distance = norm(delta);
intersections = zeros(2, 0);
if distance <= tolerance || distance > radius0 + radius1 + tolerance || ...
        distance < abs(radius0 - radius1) - tolerance
    return
end

along = (radius0^2 - radius1^2 + distance^2)/(2*distance);
heightSquared = radius0^2 - along^2;
if heightSquared <= tolerance
    return
end
unit = delta/distance;
perpendicular = [-unit(2); unit(1)];
foot = center0 + along*unit;
height = sqrt(heightSquared);
intersections = [foot + height*perpendicular, foot - height*perpendicular];
end

function rotation = rotationMatrix(angle)
rotation = [cos(angle), -sin(angle); sin(angle), cos(angle)];
end

function value = cross2d(vector0, vector1)
value = vector0(1)*vector1(2) - vector0(2)*vector1(1);
end

function angle = wrapAngle(angle)
angle = mod(angle + pi, 2*pi) - pi;
end
