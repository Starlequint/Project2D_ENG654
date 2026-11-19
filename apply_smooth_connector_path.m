model = 'Kinematically_Redundant_Planar_Parallel_Manipulator_rotation';
period = 5.76;
[time, position, velocity, acceleration, kineticEnergy] = smooth_connector_path(period);
fprintf('Mean branch kinetic energy: %.6g J; peak: %.6g J\n', ...
    mean(kineticEnergy),max(kineticEnergy));

% Report direction reversals so the new path can be compared with the old one.
jointNumbers = [20 19 18];
for leg = 1:3
    d = diff(position(:, leg));
    d = d(abs(d) > 1e-8);
    fprintf('Connector joint %d: %d reversals\n', ...
        jointNumbers(leg), sum(sign(d(2:end)) ~= sign(d(1:end-1))));
end

open_system(model);
workspace = get_param(model, 'ModelWorkspace');
signalNames = {'position','velocity','acceleration'};
signals = {position,velocity,acceleration};
for leg = 1:3
    for k = 1:3
        variableName = sprintf('connectorJoint%d_%s',jointNumbers(leg),signalNames{k});
        assignin(workspace,variableName,timeseries(signals{k}(:,leg),time));
    end
end
set_param(model,'StopTime',num2str(period,'%.15g'), ...
    'MaxStep',num2str(period/1440,'%.15g'));

% Project geometry is stored in the base workspace by the accompanying model.
AB=0.2; BC=0.2; CD=0.2; DE=0.2; EA=0.2; CF=0.2; FF=0.2;
A1A2t=2*pi/3;
A1A2x=(0.5+cos(pi/3)+2*cos(pi/6)+cos(5*pi/6-2*pi/5))*AB;
A1A2y=(sin(2*pi/5)+sin(4*pi/5)+1+sin(pi/3)+sin(2*pi/3)+ ...
    sin(2*pi/3+2*pi/5))*AB;
A1A3x=A1A2x; A1A3y=A1A2y; A1A3t=A1A2t;
A1A5x=-(0.5+cos(pi/3)+cos(pi/6)+ ...
    cos(4*pi/5-pi/3+cos(2*pi/5-pi/3)))*AB;
A1A5y=A1A2y+sin(2*pi/3)*AB; A1A5t=-2*pi/3; OA=0; EO=EA;
for name={'AB','BC','CD','DE','EA','CF','FF','A1A2t','A1A2x', ...
        'A1A2y','A1A3x','A1A3y','A1A3t','A1A5x','A1A5y','A1A5t','OA','EO'}
    assignin('base',name{1},eval(name{1}));
end
for motor=[2 4 6]
    for name={'position','velocity','acceleration'}
        assignin('base',sprintf('robotMotor%d_%s',motor,name{1}), ...
            timeseries(zeros(size(time)),time));
    end
end

set_param(model,'SimulationCommand','update');
sim(model);
save_system(model,[model '.slx']);
fprintf('Smooth full-turn path simulated successfully.\n');
