dhparams = [0   	 -pi/2	 0.089159   0;
            0.425	 0       0.0     0;];
ur5_model_p3 = rigidBodyTree('MaxNumBodies', 2,'DataFormat', 'column');
bodies = cell(2,1);
joints = cell(2,1);
for i = 1:2
    bodies{i} = rigidBody(['body' num2str(i)]);
    joints{i} = rigidBodyJoint(['jnt' num2str(i)],"revolute");
    setFixedTransform(joints{i},dhparams(i,:),"dh");
    bodies{i}.Joint = joints{i};
    if i == 1 % Add first body to base
        addBody(ur5_model_p3,bodies{i},"base")
    else % Add current body to previous body by name
        addBody(ur5_model_p3,bodies{i},bodies{i-1}.Name)
    end
end