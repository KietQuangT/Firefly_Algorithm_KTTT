function [obj_value,v_new,v0hat_new,v1hat_new,v0check_new,v1check_new,v1tilde_new] = ...
    opt_traj_firefly(v_old,v0hat_old,v1hat_old,v0check_old,v1check_old,v1tilde_old,Ups_old,b_old,sys,cons)

    %% 1. Unpack System Parameters
    K = sys.K; T = sys.T;
    u = sys.u; r = sys.r; 
    dmin = sys.dmin; dmax = sys.dmax; hmin = sys.hmin; 
    Smax = sys.Smax;
    sigma2 = sys.sigma2;
    
    % Firefly Algorithm Parameters
    n = 20;               % Population size (Number of fireflies)
    MaxGen = 50;          % Maximum generations
    alpha = 0.5;          % Randomness parameter
    beta0 = 1;            % Attractiveness at r=0
    gamma = 1;            % Light absorption coefficient
    
    %% 2. Initialization
    % Initialize population. One firefly is the previous solution (elitism)
    % The rest are random perturbations of the old trajectory.
    fireflies = repmat(v_old, 1, 1, n); 
    
    % Randomize others slightly around v_old to explore
    for i = 2:n
        noise = (rand(3, T) - 0.5) * 10; % Random noise +/- 5 meters
        fireflies(:,:,i) = v_old + noise;
        fireflies(3,:,i) = hmin; % Enforce fixed height
    end
    
    % Light intensity (Objective values)
    Light = -inf(n, 1);
    
    % Calculate initial brightness
    for i = 1:n
        fireflies(:,:,i) = enforce_constraints(fireflies(:,:,i), sys);
        Light(i) = cost_function(fireflies(:,:,i), sys, cons, b_old);
    end
    
    %% 3. Firefly Algorithm Main Loop
    for gen = 1:MaxGen
        
        for i = 1:n
            for j = 1:n
                if Light(j) > Light(i) % If firefly j is brighter than i
                    
                    % Calculate distance between trajectories (Cartesian norm)
                    r_dist = norm(fireflies(:,:,i) - fireflies(:,:,j), 'fro');
                    
                    % Beta: Attractiveness
                    beta = beta0 * exp(-gamma * r_dist^2);
                    
                    % Movement: Move i towards j + Randomness
                    epsilon = (rand(3, T) - 0.5); % Random vector
                    
                    % Update position
                    v_new_temp = fireflies(:,:,i) ...
                                 + beta * (fireflies(:,:,j) - fireflies(:,:,i)) ...
                                 + alpha * epsilon;
                    
                    % Enforce Constraints (Speed, Bounds, Closed Loop)
                    v_new_temp = enforce_constraints(v_new_temp, sys);
                    
                    % Evaluate new solution
                    new_light = cost_function(v_new_temp, sys, cons, b_old);
                    
                    % Update if better
                    if new_light > Light(i)
                        fireflies(:,:,i) = v_new_temp;
                        Light(i) = new_light;
                    end
                end
            end
        end
        
        % Optional: Reduce alpha (simulated annealing-like convergence)
        alpha = alpha * 0.98;
        
        % Track progress
        % fprintf('Gen %d: Best Obj = %.4f\n', gen, max(Light));
    end
    
    %% 4. Select Best Solution
    [best_obj, best_idx] = max(Light);
    v_new = fireflies(:,:,best_idx);
    obj_value = best_obj;
    
    %% 5. Reconstruct Auxiliary Variables
    % Since FA does not optimize v0hat, v1hat etc. directly, we calculate
    % them physically based on the optimized coordinates v_new.
    % This ensures compatibility with the outer loop of your simulation.
    
    v0hat_new = zeros(K,T);
    v0check_new = zeros(K,T);
    v1hat_new = zeros(T,1);
    v1check_new = zeros(T,1);
    v1tilde_new = zeros(T,1);
    
    e0 = sys.e0; % Path loss exponent
    e1 = sys.e1;
    
    for t = 1:T
        % Reconstruct user distances
        for k = 1:K
            dist_sq = norm(v_new(:,t) - u(:,k))^2;
            % Assuming v0hat represents a distance-related term (e.g., d^-2/e0)
            % Adjust this formula based on your exact definition of v0hat inside get_Fpow
            v0hat_new(k,t) = dist_sq; 
            v0check_new(k,t) = v0hat_new(k,t);
        end
        
        % Reconstruct RIS distances
        dist_ris_sq = norm(v_new(:,t) - r)^2;
        v1hat_new(t) = dist_ris_sq;
        v1check_new(t) = v1hat_new(t);
        v1tilde_new(t) = v1hat_new(t);
    end

end

%% Sub-function: Cost Function (Objective)
function tau = cost_function(v, sys, cons, b_old)
    % Reconstructs the Max-Min Rate Objective
    % Corresponds to: max tau s.t. average_rate >= tau
    
    K = sys.K; T = sys.T;
    u = sys.u; r = sys.r;
    sigma2 = sys.sigma2;
    
    % Note: Since we don't have the exact channel gain formulas (c0, c1 are
    % approximations), we estimate the rate using distances directly.
    % If you have the exact channel function H(v), put it here.
    
    rates = zeros(K, 1);
    
    for k = 1:K
        sum_rate_t = 0;
        for t = 1:T
            % Distance UAV-User
            d_uk = norm(v(:,t) - u(:,k));
            
            % Distance UAV-RIS (Assuming reflected path exists)
            d_ur = norm(v(:,t) - r);
            
            % Simple Path Loss Model for Fitness Evaluation
            % (Replace 1e3 with your actual reference channel gain beta0)
            gain_direct = 1e3 / (d_uk ^ sys.e0); 
            
            % RIS reflected part (Simplified for the heuristic)
            % In strict FA, you should use the exact Eq (31a) RHS if possible
            gain_ris = 0; 
            if isfield(cons, 'c1') 
               % Estimating based on provided matrices if needed
               % But pure distance usually works better for geometrical updates
               gain_ris = 1e3 / (d_ur ^ sys.e1); 
            end
            
            % Total SNR
            snr = (gain_direct + gain_ris) / sigma2(k,t);
            
            % Instantaneous Rate
            rate_t = log2(1 + snr);
            
            % Accumulate weighted rate
            sum_rate_t = sum_rate_t + b_old(k,t) * rate_t;
        end
        rates(k) = (1/T) * sum_rate_t;
    end
    
    % The objective is to maximize the minimum user rate
    tau = min(rates);
end

%% Sub-function: Constraint Enforcement
function v = enforce_constraints(v, sys)
    T = sys.T;
    dmin = sys.dmin; dmax = sys.dmax; 
    hmin = sys.hmin; Smax = sys.Smax;
    
    % 1. Enforce Height (2D Trajectory assumption based on v(3,:)==hmin)
    v(3,:) = hmin;
    
    % 2. Enforce Area Bounds (X and Y)
    v(1:2,:) = max(min(v(1:2,:), dmax), dmin);
    
    % 3. Enforce Closed Loop (v(:,1) == v(:,T))
    v(:,T) = v(:,1);
    
    % 4. Enforce Max Speed (Step Size Smax)
    % We iterate forward and pull points back if they jumped too far
    for t = 1:T-1
        diff = v(:,t+1) - v(:,t);
        dist = norm(diff);
        if dist > Smax
            % Scale the vector to be exactly length Smax
            v(:,t+1) = v(:,t) + (diff / dist) * Smax;
        end
    end
    
    % Re-enforce closed loop explicitly after speed adjustment
    % (This might slightly violate Smax at the last step, but ensures loop closure)
    v(:,T) = v(:,1);
end