function visual_box(fig,A,B,P)

figure(fig);
% hold on;
P1 = A([1,2],:)\B([1,2],:)+P;
P2 = A([2,3],:)\B([2,3],:)+P;
P3 = A([3,4],:)\B([3,4],:)+P;
P4 = A([4,1],:)\B([4,1],:)+P;

V = [P1';P2';P3';P4'];
F = [1 2 3 4];
patch('Faces',F,'Vertices',V,'FaceColor','none')
axis equal;
end
