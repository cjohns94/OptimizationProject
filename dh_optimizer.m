% DH Parameter optimzer for a given set of task poses. 
% 
% This function seeks to minimize the sum of the norms of each pose error
% represented as a 6x1 column vector (see pg 67 in robotics txt) subject to a
% maximum arm length and maximum allowable joint torques.
% 
% Objective:
%     sum(norm(pose_task_i - pose_arm_i)) for i = 1:p (number of tasks)
% Constraints:
%     sum(d_i + a_i) <= max_length
%     max(Q) <= max_torque where Q = J.' * W (W is a wrench applied at the ee)
% 
% Inputs: 
%     robot - SerialLink Object with initial DH params from Peter Corke's Robotics Toolbox
%     poses - 4x4xp matrix where p is the number of poses required to define
%             desired workspace. Note that each of these poses should be
%             relative to the world (base) frame of the robot. 
% Outputs:
%     robot_opt - SerialLink object containing optimal dh parameters to reach each task pose
function robot_opt = dh_optimizer(robot, poses)
    tic
    q_curr = zeros(1,6);
    DOF = robot.n;
% 
%     if isempty(robot.theta)
%         theta = zeros(robot.n, 1);
%     else
%         theta = robot.theta;
%     end
        
    %DH parameters of robot, need to be in column vector.
    %[d(1:n); a(1:n); alpha(1:n); offset(1:n)]
    x0 = [robot.d.'; robot.a.'; robot.alpha.'; robot.offset.']; %maybe no offset?

    %Set bounds and options needed for fmincon  
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    
    %Column vectors for lower bounds
    d_lb = zeros(DOF,1)-.0001;
    a_lb = zeros(DOF,1)-.00101;
    alpha_lb = ones(DOF,1)*-2*pi;
    offset_lb = ones(DOF,1)*-2*pi;    
    lb= [d_lb; a_lb; alpha_lb; offset_lb]; 
    
    %Column vectors for lower bounds
    d_ub = ones(DOF,1)*10;
    a_ub = ones(DOF,1)*10;
    alpha_ub = ones(DOF,1)*2*pi;
    offset_ub = ones(DOF,1)*2*pi;    
    ub= [d_ub; a_ub; alpha_ub; offset_ub]; 


% ----------- options ----------------------------
    options = optimoptions('fmincon', ...
        'Algorithm', 'sqp', ...  % choose one of: 'interior-point', 'sqp', 'active-set', 'trust-region-reflective'
        'HonorBounds', true, ...  % forces optimizer to always satisfy bounds at each iteration
        'Display', 'iter-detailed', ...  % display more information
        'MaxIterations', 1000, ...  % maximum number of iterations
        'MaxFunctionEvaluations', 1000, ...  % maximum number of function calls
        'OptimalityTolerance', 1e-6, ...  % convergence tolerance on first order optimality
        'ConstraintTolerance', 1e-8, ...  % convergence tolerance on constraints
        'FiniteDifferenceType', 'forward', ...  % if finite differencing, can also use central
        'SpecifyObjectiveGradient', false, ...  % supply gradients of objective
        'SpecifyConstraintGradient', false, ...  % supply gradients of constraints
        'CheckGradients', false, ...  % true if you want to check your supplied gradients against finite differencing
        'Diagnostics', 'on');
    
    [x_opt, f_opt, ouput] = fmincon(@(x) pose_error(x, poses), x0, A,b,Aeq, beq, lb, ub, @constraints, options);
    
    %Reconstruct robot here -- jangles and stuff come from output
    d_opt = x_opt(1:6);
    a_opt = x_opt(7:12);
    alpha_opt = x_opt(13:18);
    offset_opt = x_opt(19:24);
    
    %Make new robot iteration with optimal DH params
    for k = 1:DOF
        links_opt(k) = Revolute('d', d_opt(k), 'a', a_opt(k), 'alpha', alpha_opt(k), 'offset', offset_opt(k));
    end
    
    robot_opt = SerialLink(links_opt);
    toc

    function J = pose_error(x, poses)
        error = 0;        
        d = x(1:6);
        a = x(7:12);
        alpha = x(13:18);
        offset = x(19:24);
        
        %Make new robot iteration with current DH params
        for j = 1:DOF
            links(j) = Revolute('d', d(j), 'a', a(j), 'alpha', alpha(j), 'offset', offset(j));
        end
        
        robot_iter = SerialLink(links); %might make this global to access from constraint func too...
        %check size of poses matrix
        temp = length(size(poses));
        if temp < 3
            p = 1;
        else
            p = size(poses,3); %3rd dimension is depth
        end
        
        for i = 1:p %length of poses is depth (4x4xdepth)
            %calculate inverse kinematics
            [q_curr,pose_error] = robot_iter.ikine_modified(poses(:,:,i));
            %update error
            error = error + pose_error;
        end
        J = error;
    end

    function [c,ceq] = constraints(x)
        % c<=0 and ceq <=0 convention
        max_length = 2;
        %unpack x
        d = x(1:6);
        a = x(7:12);
        alpha = x(13:18);
        offset = x(19:24);
        
        %Calculate manip jacobian at given configuration
        %Make new robot iteration with current DH params
        for j = 1:DOF
            links(j) = Revolute('d', d(j), 'a', a(j), 'alpha', alpha(j), 'offset', offset(j));
        end
        
        robot_iter = SerialLink(links);
        
        jacobian = robot_iter.jacob0(q_curr);
        W = [20 0 0 0 0 0].';
        joint_torques = jacobian.'*W;
        
        c = zeros(2,1);
%         c = [];
        c(1) = sum(d) + sum(a) - max_length;
        c(2) = max(abs(joint_torques)) - 10;
        %in order to get joint torques, I need configuration...
        ceq = []; % no equality constraints
    end
end
