% Firefly Algorithm (FA)
clear; clc; close all;

%% ================== Tham số thuật toán ==================
n = 20;             % Số đom đóm
MaxGen = 100;       % Số thế hệ
alpha = 0.25;       % Tham số random (mutation)
beta0 = 1;          % Độ hấp dẫn ban đầu
gamma = 1;          % Hệ số suy giảm ánh sáng
dim = 2;            % Số chiều bài toán
LB = -10 * ones(1, dim); 
UB =  10 * ones(1, dim);

%% ============ Khởi tạo quần thể đom đóm =================
x = rand(n, dim) .* (UB - LB) + LB;
I = zeros(n,1);     % Độ sáng
for i = 1:n
    I(i) = fitness(x(i,:));
end

%% ======== Tìm đom đóm sáng nhất ban đầu (Global Best) ===
[globalBestVal, idx] = min(I);
globalBest = x(idx,:);

%% ===================== Vòng lặp chính ====================
for t = 1:MaxGen
    
    for i = 1:n
        for j = 1:n
            if I(j) < I(i)   % j sáng hơn i  → i di chuyển về j
                r = norm(x(i,:) - x(j,:));
                beta = beta0 * exp(-gamma * r^2);  % độ hấp dẫn
                
                % Công thức cập nhật vị trí
                x(i,:) = x(i,:) + beta * (x(j,:) - x(i,:)) ...
                         + alpha * (rand(1,dim) - 0.5);
            end
        end
        
        % Giữ trong giới hạn
        x(i,:) = max(x(i,:), LB);
        x(i,:) = min(x(i,:), UB);

        % Cập nhật lại độ sáng
        I(i) = fitness(x(i,:));
    end
    
    % Cập nhật Global Best
    [bestVal, idx] = min(I);
    if bestVal < globalBestVal
        globalBestVal = bestVal;
        globalBest = x(idx,:);
    end
    
    fprintf("Gen %d → Best = %.6f\n", t, globalBestVal);
end

%% =============== Kết quả ====================
disp("Global Best Position:");
disp(globalBest);
disp("Best Fitness:");
disp(globalBestVal);


%% ======== Hàm Fitness (tối thiểu hoá) =============
function y = fitness(x)
    % Sphere function: f(x)=sum(x^2) — dễ quan sát
    y = sum(x.^2);
end
