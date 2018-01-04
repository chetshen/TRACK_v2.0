%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%Main program: dynamic analysis
%%%Author: Chen Shen
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clear;
% %%
% %%build track model
disp (['Start assembling system matrix. Time: ' datestr(now)]);
tic;
TRACK_main2;
if flag==1
    return
end

disp (['Matrix assembly complete. Time used: ', num2str(toc),' s']);
%%
%%initial conditions
acc.r=zeros(inp.solver.n_ts+1,length(mat_trk.K_reduced));
vel.r=zeros(inp.solver.n_ts+1,length(mat_trk.K_reduced));
dis.r=zeros(inp.solver.n_ts+1,length(mat_trk.K_reduced));
acc.w=zeros(inp.solver.n_ts+1,1);
vel.w=zeros(inp.solver.n_ts+1,1);
dis.w=zeros(inp.solver.n_ts+1,1);
Z.w=zeros(inp.solver.n_ts+1,1);
Z.r=zeros(inp.solver.n_ts+1,1);
Z.irr=zeros(inp.solver.n_ts+1,1); %can be read in with files
F=zeros(inp.solver.n_ts+1,1);
X_w=zeros(inp.solver.n_ts+1,1);
X_w(1,1)=15;   %initial x coordinates of wheel
vx=inp.solver.Vx; %vehicle speed
contactID=5; %5 for non-linear  10 for linear 8 for winkler bedding 

%%
%%irregularity definition
prompt='Please select the irregularity definition(1.Sinsoidal;2.Input file: [1]\n';

i=input(prompt);
flag=0;
if isempty(i)
    i=1;
end

switch i
    case 1
        %%irregularity definition: squat G302 maria
        irr_depth=0.3e-3;
        irr_length=50e-3;
        irr_x0=15.485; %30.38=15.38=0.98 in FE 15.5=1.1
        irr_ts0=round((irr_x0-X_w(1,1))/vx/inp.solver.deltat);
        irr_ts1=round((irr_x0-X_w(1,1)+1*irr_length)/vx/inp.solver.deltat);
        
        %%irregularity definition: wheelflat benchmark
%         irr_depth=0.3e-3;
%         irr_length=50e-3;
%         irr_x0=30.575;
%         irr_ts0=round((irr_x0-X_w(1,1))/vx/inp.solver.deltat);
%         irr_ts1=round((irr_x0-X_w(1,1)+irr_length)/vx/inp.solver.deltat);
%         
%         
        
        for i=irr_ts0:1:irr_ts1;           
            Z.irr(i,1)=irr_depth./2*(cos(2*pi./irr_length*(vx*inp.solver.deltat*i-(irr_x0-X_w(1,1))))-1);
        end
        
    case 2
        %%irregularity definition: measured
        load('measured_geometry_squat_Molodova_2014.mat', 'irr');
        irr_length=215e-3;
        irr_x0=15.4; %30.38=0.98 in FE
        irr(:,3)=irr_x0+irr(:,1);
        irr_ts0=round((irr_x0-X_w(1,1))/vx/inp.solver.deltat);
        irr_ts1=round((irr_x0-X_w(1,1)+irr_length)/vx/inp.solver.deltat);
        
        for i=irr_ts0:1:irr_ts1;
            xq=vx*inp.solver.deltat*i+X_w(1,1);
            
            Z.irr(i,1)=interp1(irr(:,3),irr(:,2),xq,'linear','extrap');
        end
    case 3
        %%irregularity definition: half sine
        irr_depth=0.2e-3;
        irr_length=30e-3;
        irr_x0=15.28; %30.38=15.38=0.98 in FE 15.5=1.1
        irr_ts0=round((irr_x0-X_w(1,1))/vx/inp.solver.deltat);
        irr_ts1=round((irr_x0-X_w(1,1)+irr_length)/vx/inp.solver.deltat);
        
       
        
        for i=irr_ts0:1:irr_ts1;           
            Z.irr(i,1)=-irr_depth.*(sin(pi./irr_length*(vx*inp.solver.deltat*i-(irr_x0-X_w(1,1)))));
        end
    case 4
        %%irregularity definition: dipped joint
        irr_depth=3.5e-3;
        irr_length=1;
        irr_x0=15.1; %30.38=15.38=0.98 in FE 15.5=1.1
        irr_ts0=round((irr_x0-X_w(1,1))/vx/inp.solver.deltat);
        irr_ts1=round((irr_x0-X_w(1,1)+irr_length)/vx/inp.solver.deltat);
        
        for i=irr_ts0:1:round(irr_ts1/2);
            Z.irr(i,1)=-irr_depth.*(vx*inp.solver.deltat*i-(irr_x0-X_w(1,1)))/(irr_length/2);
        
        end
        for i=round(irr_ts1/2)+1:irr_ts1
        Z.irr(i,1)=-irr_depth.*(irr_length-(vx*inp.solver.deltat*i-(irr_x0-X_w(1,1))))/(irr_length/2);
        end
        
end



%%
%%irregularity definition: measured
% load('measured_geometry_squat_Molodova_2014.mat', 'irr'); 
% irr_length=200e-3;
% irr_x0=30.2; %30.38=0.98 in FE
% irr(:,3)=irr_x0+irr(:,1);
% irr_ts0=round((irr_x0-X_w(1,1))/vx/inp.solver.deltat);
% irr_ts1=round((irr_x0-X_w(1,1)+irr_length)/vx/inp.solver.deltat);
% 
% for i=irr_ts0:1:irr_ts1;
% xq=vx*inp.solver.deltat*i+X_w(1,1);
% 
% Z.irr(i,1)=interp1(irr(:,3),irr(:,2),xq,'linear','extrap');
% end
%%
%%shape function for initial condition
shape_initial=form_shape_fun(geo,mat_trk,[X_w(1,1),-0.75,0]);

%static analysis
[dis_initial,Z_initial,F_initial]=solver_static(mat_trk,inp,shape_initial,contactID);
dis.r(1,:)=dis_initial.r;
dis.w(1,1)=dis_initial.w;
Z.r(1,1)=Z_initial.r;
Z.w(1,1)=Z_initial.w;
F(1,1)=F_initial;


prompt='Moving irregularity(1yes;2 no: [2]\n';
i=input(prompt);
flag=0;
if isempty(i)
    i=2;
end
    switch i
    case 1
    %
    vx=0;
    %

    case 2
    end
    
    
%%
%dynamic analysis
disp (['Starting Newmark intergration. Time: ' datestr(now)]);
tic;
for i=1:inp.solver.n_ts
    X_w(i+1,1)=X_w(1,1)+i*inp.solver.deltat*vx;
    shape=form_shape_fun(geo,mat_trk,[X_w(i+1,1),-0.75,0]);
    
    acc1.r=acc.r(i,:);
    vel1.r=vel.r(i,:);
    dis1.r=dis.r(i,:);
    acc1.w=acc.w(i,1);
    vel1.w=vel.w(i,1);
    dis1.w=dis.w(i,1);
    position.w=Z.w(i,1);
    position.r=shape*dis1.r'; 
    position.irr=Z.irr(i,1);
    
    
    [acc2,vel2,dis2,F_contact,position]=solver_newmark_iter(mat_trk,inp,shape,...
        position, inp.ext_force.wh_ld, acc1, vel1, dis1,X_w(i+1,1), geo, Z,contactID);
    
    acc.r(i+1,:)=acc2.r;
    vel.r(i+1,:)=vel2.r;
    dis.r(i+1,:)=dis2.r;
    acc.w(i+1,1)=acc2.w;
    vel.w(i+1,1)=vel2.w;
    dis.w(i+1,1)=dis2.w;
    Z.w(i+1,1)=position.w;
    Z.r(i+1,1)=position.r;
%     Z.irr(i+1,1)=position.irr;
    F(i+1,1)=F_contact;
    
    disp (['Time step: ' num2str(i) 'finished. Time' num2str(toc)]);
    
end

%%
% figure;
% plot(X_w,F);

%%
clear acc1 acc2 dis1 dis2 dis_initial i position shape vel1 vel2;