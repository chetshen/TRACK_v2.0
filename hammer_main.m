%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%Main program: dynamic analysis
%%%Author: Chen Shen
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clear;

%build track model
disp (['Start assembling system matrix. Time: ' datestr(now)]);
tic;
TRACK_main2;
disp (['Matrix assembly complete. Time used: ', num2str(toc),' s']);

[dis, vel,acc, t]=solver_newmark(inp,mat_trk,geo);


X_load=inp.ext_force.x;
[shape,Ref_Dof]=form_shape_fun(geo,mat_trk,X_load);


dis_x_load=dis*shape';
% figure;
% plot(t,dis_x_load);

[p_dis,f_dis]=periodogram(dis_x_load(2:length(dis_x_load)),hamming(length(dis_x_load)-1),length(dis_x_load)-1,inp.ext_force.sf);

%[p_dis,f_dis]=periodogram(dis_x_load(1:length(dis_x_load)),hamming(length(dis_x_load)),length(dis_x_load),inp.ext_force.sf);
force=load(inp.ext_force.timeh);
[p_force,f_force]=periodogram(force,hamming(length(force)),length(force),inp.ext_force.sf);

acc_x_load=acc*shape';
[p_acc,f_acc]=periodogram(acc_x_load(2:length(acc_x_load)),hamming(length(acc_x_load)-1),length(acc_x_load)-1,inp.ext_force.sf);

% figure;
% plot(f_dis,p_dis./p_force);
% dis=dis(:,Ref_Dof);
% acc=acc(:,Ref_Dof);
% vel=vel(:,Ref_Dof);

% clear force geo mat_trk ;






