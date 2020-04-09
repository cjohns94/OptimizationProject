%This script generates poses that are required for a specific task (pick and
%place, painting, etc).
%The poses need to be 4x4 homogenous transform matrices.

num_poses = 2;

%Package up poses - needs to be 4 x 4 x # of poses.
task_poses = zeros(4,4,num_poses);

%pose 1
pose1 = transl(1.5,0.5,0.5) * troty(pi/2) * trotz(-pi/2);
task_poses(:,:,1) = pose1;

%pose 2
pose2 = transl(1.5,-0.5,-0.5) * troty(pi/2) * trotz(-pi/2);
task_poses(:,:,2) = pose2;
