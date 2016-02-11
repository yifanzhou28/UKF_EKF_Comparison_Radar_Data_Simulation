figure(999);
clf
hold on;
axis([-500, 2000, -500, 2000]);
plot(store_Target_state(1, :), store_Target_state(2, :), 'c.');

for i = 1:size(Target_range, 2)
    relative_pos = [Target_range(i)*cos(Target_ori(i)); Target_range(i)*sin(Target_ori(i))];
    plot(store_Observer_state(1, i)+relative_pos(1), store_Observer_state(2, i)+relative_pos(2), 'r.');
    plot(store_Target_state(1, i), store_Target_state(2, i), 'k.');
    pause(0.01)
end
