sd_ori = 2e-2;
sd_range = 1e-3;

Target_ori = [];
Target_range = [];

for i = 1:size(store_Target_real_state, 2)
    %relative_pos = [store_Target_state(1, i)-store_Observer_state(1, i); store_Target_state(2, i)-store_Observer_state(2, i)];
    Target_ori = [Target_ori, normrnd(store_Target_real_ori(1, i), sd_ori)];
    Target_range = [Target_range, normrnd(store_Target_real_range(1, i), sd_range)];
    
end
Observations = [Target_range; Target_ori];
