%constang velocity
T = 1;
F = [1 0 T 0;...
    0 1 0      T;...
    0 0 1      0;...
    0 0 0      1];


%% Target
store_Target_real_state = [];
Target_state = [2000; 2000; 0.2; 0.3];
Target_acceleration = [0; 0; 0; 0];

%% Observer 1
store_Observer_state = [];
store_Target_real_ori = [];
store_Target_real_range = [];
Observer_state = [0; 0; 0; 0];

%% Generate Data
% Target
store_Target_real_state = [store_Target_real_state, Target_state];

% Observer 1
store_Observer_state = [store_Observer_state, Observer_state];
relative_pos = [Target_state(1)-Observer_state(1); Target_state(2)-Observer_state(2)];

% Orientation
Target_real_ori = atan2( relative_pos(2), relative_pos(1) );
store_Target_real_ori = [store_Target_real_ori, Target_real_ori];

% Range
Target_real_range = norm(relative_pos);
store_Target_real_range = [store_Target_real_range, Target_real_range];

ProcessNoiseCov = 1e-5 * [...
    1/3*T^3, 0,                      1/2*T^2, 0;...
    0,                      1/3*T^3, 0,                     1/2*T^2;...
    1/2*T^2, 0,                       T,       0;...
    0,                      1/2*T^2,  0,                     T];


%go straight
for t = 1:999
    randPN = mvnrnd([0, 0, 0, 0], ProcessNoiseCov)';
    % Target
    Target_state = F*Target_state + randPN;
    store_Target_real_state = [store_Target_real_state, Target_state];
    
    % Observer
    Observer_state = F*Observer_state;
    store_Observer_state = [store_Observer_state, Observer_state];
    
    % Relative Position
    relative_pos = [Target_state(1)-Observer_state(1); Target_state(2)-Observer_state(2)];
    
    % Orientation
    Target_real_ori = atan2( relative_pos(2), relative_pos(1) );
    store_Target_real_ori = [store_Target_real_ori, Target_real_ori];
    
    % Range
    Target_real_range = norm(relative_pos);
    
    store_Target_real_range = [store_Target_real_range, Target_real_range];
    
end
