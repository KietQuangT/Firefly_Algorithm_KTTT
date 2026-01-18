    function [obj_value,v_new,v0hat_new,v0check_new] = opt_traj_noRIS_firefly(v_old,v0hat_old,v0check_old,b_old,sys,cons)
    
    %% 1. Unpack System Parameters
    K = sys.K; T = sys.T;
    e0 = sys.e0; % Path loss exponent
    u = sys.u;   % User locations
    
    dmin = sys.dmin; dmax = sys.dmax; 
    hmin = sys.hmin; 
    Smax = sys.Smax; % Max speed
    sigma2 = sys.sigma2;
    
    % Firefly Algorithm Parameters
    n = 20;               % Số lượng đom đóm (Population size)
    MaxGen = 50;          % Số vòng lặp tối đa (Generations)
    alpha = 0.2;          % Hệ số ngẫu nhiên (Randomness)
    beta0 = 1;            % Độ thu hút cơ bản (Attractiveness)
    gamma = 1.0;          % Hệ số hấp thụ ánh sáng
    
    %% 2. Initialization
    % Khởi tạo quần thể. Con đầu tiên là kết quả cũ (Elitism)
    fireflies = repmat(v_old, 1, 1, n); 
    
    % Tạo các con còn lại bằng cách làm nhiễu nhẹ quỹ đạo cũ
    for i = 2:n
        noise = (rand(3, T) - 0.5) * 5; % Nhiễu ngẫu nhiên +/- 2.5m
        fireflies(:,:,i) = v_old + noise;
    end
    
    % Tính độ sáng ban đầu (Light Intensity)
    Light = -inf(n, 1);
    for i = 1:n
        fireflies(:,:,i) = enforce_constraints(fireflies(:,:,i), sys);
        Light(i) = get_obj_value(fireflies(:,:,i), sys, b_old);
    end
    
    %% 3. Main Loop
    for gen = 1:MaxGen
        for i = 1:n
            for j = 1:n
                if Light(j) > Light(i) % Nếu con j sáng hơn con i -> Di chuyển i về phía j
                    
                    % Khoảng cách giữa 2 quỹ đạo
                    r_dist = norm(fireflies(:,:,i) - fireflies(:,:,j), 'fro');
                    
                    % Beta: Độ thu hút giảm dần theo khoảng cách
                    beta = beta0 * exp(-gamma * r_dist^2);
                    
                    % Di chuyển: Cũ + Hướng về j + Ngẫu nhiên
                    epsilon = (rand(3, T) - 0.5); 
                    v_temp = fireflies(:,:,i) ...
                             + beta * (fireflies(:,:,j) - fireflies(:,:,i)) ...
                             + alpha * epsilon;
                    
                    % Ràng buộc vật lý (Vận tốc, biên, vòng kín)
                    v_temp = enforce_constraints(v_temp, sys);
                    
                    % Đánh giá lại mục tiêu
                    new_light = get_obj_value(v_temp, sys, b_old);
                    
                    % Cập nhật nếu tốt hơn
                    if new_light > Light(i)
                        fireflies(:,:,i) = v_temp;
                        Light(i) = new_light;
                    end
                end
            end
        end
        
        % Giảm dần độ ngẫu nhiên để hội tụ
        alpha = alpha * 0.98;
    end
    
    %% 4. Get Best Result
    [obj_value, best_idx] = max(Light);
    v_new = fireflies(:,:,best_idx);
    
    %% 5. Reconstruct Auxiliary Variables (Output Compatibility)
    % Tính toán lại v0hat để khớp với output của hàm gốc
    v0hat_new = zeros(K,T);
    for t = 1:T
        for k = 1:K
            % v0hat trong bài toán gốc thường liên quan đến khoảng cách bình phương
            % hoặc d^(-2/e0). Ở đây mình gán distance^2 để tượng trưng.
            v0hat_new(k,t) = norm(v_new(:,t) - u(:,k))^2;
        end
    end
    v0check_new = 0; % Biến này không dùng trong NoRIS heuristic

end

%% --- Helper Function: Objective Function ---
function tau = get_obj_value(v, sys, b_old)
    % Hàm mục tiêu: Max-Min Average Rate
    K = sys.K; T = sys.T;
    u = sys.u; 
    e0 = sys.e0;
    sigma2 = sys.sigma2;
    
    rates = zeros(K, 1);
    
    % Giả lập kênh truyền (Simplified Channel Model)
    % Lưu ý: Cần điều chỉnh ref_gain (1e3) cho khớp với simulation của bạn
    ref_gain = 1000; 
    
    for k = 1:K
        sum_rate = 0;
        for t = 1:T
            dist = norm(v(:,t) - u(:,k));
            
            % Tránh chia cho 0
            if dist < 0.1, dist = 0.1; end
            
            % SNR = P * Gain / Noise
            channel_gain = ref_gain / (dist ^ e0);
            snr = channel_gain / mean(sigma2(:)); % Lấy mean noise cho đơn giản
            
            rate_t = log2(1 + snr);
            sum_rate = sum_rate + b_old(k,t) * rate_t;
        end
        rates(k) = (1/T) * sum_rate;
    end
    
    tau = min(rates); % Maximize the minimum rate
end

%% --- Helper Function: Enforce Constraints ---
function v = enforce_constraints(v, sys)
    T = sys.T;
    dmin = sys.dmin; dmax = sys.dmax;
    hmin = sys.hmin; Smax = sys.Smax;
    
    % 1. Cố định độ cao (2D Trajectory)
    v(3,:) = hmin;