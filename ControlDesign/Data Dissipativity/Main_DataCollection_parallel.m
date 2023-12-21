%% Main_DataCollection_parallel
%  Ethan LoCicero
%  December 5, 2023

% This code implements the data-based gain identification example from
% Montenbruck and Allgower 2016 using parallel processing.

tic
%% Setup
% Inputs
M               = 15;   % The number of sampled trajectories is M^d 
stabilizer_gain = 1;    % Coefficient on order 1/sqrt(delta) stabilizer
d               = 4;    % basis dimension
b               = 10;   % input bound
T               = 1;    % time interval
dt              = .001; % simulation step size

% Legendre Polynomial basis functions
Ntime = T/dt+1;
t     = linspace(0,T,Ntime);
v1    = 1;
v2    = 2*t - 1;
v3    = 6*t.^2 - 6*t + 1;
v4    = 20*t.^3 - 30*t.^2 + 12*t - 1;

% Dynamics hyperparameters
Nstate  = 2;                   % state dimension
Noutput = 1;                   % output dimension
Ninput  = 1;                   % input dimension
x0      = zeros(Nstate,1);     % initial conditions

%% Generate Grid
Delta = 2*b/((M+1));             % grid spacing
weight = -b+Delta:Delta:b-Delta; % basis weights
Ngrid = length(weight);          % number of samples along each basis

%% Calculate Covering Radius
delta = .5*d*Delta;

%% Calculate inputs on grid and simulate outputs 
% initialize data matrices
Nsimulations = Ngrid^d;
Nd1 = Ngrid^(d-1);
Nd2 = Ngrid^(d-2);
Nd3 = Ngrid^(d-3);
u     = zeros(Ngrid^d,Ninput,Ntime);
x     = zeros(Nstate,Ntime);
y     = zeros(Ngrid^d,Noutput,Ntime);
% collect data at each node
parfor index = 0:Nsimulations-1
    % Generate Input
    i = floor(  index                         / Nd1 );
    j = floor( (index - i*Nd1)                / Nd2 );
    k = floor( (index - i*Nd1 - j*Nd2)        / Nd3 );
    l = floor(  index - i*Nd1 - j*Nd2 - k*Nd3       );
    weight_i = weight(i+1);
    weight_j = weight(j+1);
    weight_k = weight(k+1);
    weight_l = weight(l+1);
    u(index+1,:,:) = weight_i.*v1 + weight_j.*v2 + weight_k.*v3 + weight_l.*v4;
    % Generate Output
    [~,x]          = ode45(@(t,x) rhs(t,x,weight_i,weight_j,weight_k,weight_l) , t , x0);
    y(index+1,:,:) = x(:,1)';
end

%% Calculate Lipshitz Constant
L = zeros(1,Nsimulations^2); % Lipshitz Constant
% For every sample trajectory...
parfor index = 1:Nsimulations^2
    idx1 = ceil(index/Nsimulations);
    idx2 = mod(index,Nsimulations)+1;
    if idx1 > idx2
        u1 = u(idx1,:,:);
        y1 = y(idx1,:,:);
        u2 = u(idx2,:,:);
        y2 = y(idx2,:,:);
        % Calculate the Lipshitz Constant
        norm_du = InnerProduct(u1-u2,dt);
        norm_dy = InnerProduct(y1-y2,dt);
        L(index) = norm_dy/norm_du;
    end
end
L = max(L);

%% Estimate Gain
norm_G = zeros(1,Nsimulations); % System Gain
stabilizer = delta + stabilizer_gain/sqrt(delta);
% For every trajectory
parfor index = 1:Nsimulations
    % For all inputs greater than delta
    norm_u = InnerProduct(u(index,:,:),dt);
    if norm_u > stabilizer
        % Calculate system gain
        norm_y = InnerProduct(y(index,:,:),dt);
        norm_G(index) = (L*delta + norm_y)/(norm_u - delta);

    end
end
norm_G = max(norm_G);

%% Save Results
mkdir data
writematrix(norm_G,'data/SystemNorm.csv'      )
writematrix(L     ,'data/LipschitzContant.csv')
writematrix(delta ,'data/CoveringRadius.csv'  )

%% (Maybe) Save data
writematrix(T    ,'data/MaxTime.csv'        )
writematrix(dt   ,'data/StepSize.csv'       )
writematrix(d    ,'data/BasisDimension.csv' )
writematrix(b    ,'data/MaxInput.csv'       )
writematrix(Delta,'data/GrideSize.csv'      )
writematrix(M    ,'data/NumberOfSamples.csv')
writematrix(u    ,'data/InputSamples.csv'   )
writematrix(y    ,'data/OutputSamples.csv'  )
writematrix(stabilizer_gain,'data/Stabilizer.csv')

time = toc

%% Dynamics Functions
function xdot = rhs(t,x,weight_i,weight_j,weight_k,weight_l)

v1    = 1;
v2    = 2*t - 1;
v3    = 6*t.^2 - 6*t + 1;
v4    = 20*t.^3 - 30*t.^2 + 12*t - 1;
u     = weight_i.*v1 + weight_j.*v2 + weight_k.*v3 + weight_l.*v4;

xdot = zeros(2,1);
xdot(1) = x(2);
xdot(2) = 4*(1 - x(1)^2)*x(2) - x(1) + u;

end

%% Inner Product Function
function norm = InnerProduct(x,dt)
% Justification: the inner product of a continuous signal, x(t), is
% int_0^T x'(t)x(t) dt.
% The discrete approximation is 
% sum_k x'(k)*x(k)*Deltat.
% The scalar form of this is
% Deltat * sum_k x1(k)^2 + x2(k)^2 + x3(k)^2 + ...
% Expanding the sum gives
% Deltat * [ x(1,1)^2 + x(2,1)^2 + ... + x(1,2)^2 + x(2,2)^2 + ... ]
% Which is just squaring every element, summing, and multiplying bt Dt.

norm = dt.*sum(x.^2,'all');

end