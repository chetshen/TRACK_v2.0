%iteration during one time step
% Z.w=0;
% Z.r=0;
% Z.irr=0;
% acc1.r=zeros(1,length(mat_trk.K_reduced));
% vel1.r=acc1.r;
% dis1.r=acc1.r;
% acc1.w=-9.8;
% vel1.w=-0.99;
% dis1.w=0;
% shape=acc1.r;
% shape(1,299)=1;
%nonlinear solver for vehicle track interaction within one time step using
%newmark integration and newton-raphson
function [acc2,vel2,dis2,F,Z]=solver_newmark_iter(mat_trk,inp,shape, Z, wh_ld, acc1, vel1, dis1,X_w_t, geo,Z_global,contactID, mat_vhcl)
%initial condition
etol=1e-7;
deltat=inp.solver.deltat;
m_w=inp.mater(6).Data(2);
M_trk=mat_trk.M_reduced;
K_trk=mat_trk.K_reduced;
% current_ts=X_w_t;


if isfield(mat_trk,'C_reduced')==1
    C_trk=mat_trk.C_reduced;
else
    C_trk=sparse(zeros(length(mat_trk.K_reduced),length(mat_trk.K_reduced)));
end

%initial condition for iteration
ite=0;
penetration = Z.w-Z.r-Z.irr;
if penetration > 0
    penetration=0;
end

switch contactID
    case 5
        %non-linear
        F = inp.mater(contactID).Data*(abs(penetration).^1.5);
    case 10
        %linear
        
        F = inp.mater(contactID).Data*abs(penetration);
end


while 1
    
    F0=F;
    
    %for track system: newmark integration
    R_trk=-F*shape';
    [dis2.r, vel2.r,acc2.r]=newmark_sub(K_trk,M_trk,C_trk,R_trk,dis1.r,vel1.r, acc1.r, deltat);
    Z.r=shape*dis2.r'; %modification needed
    
    %for vehicle system
    %    rigid wheelset;
    acc2.w=(-m_w*9.8-wh_ld+F)/m_w;
    vel2.w=vel1.w+deltat/2*(acc1.w+acc2.w);
    dis2.w=dis1.w+deltat/2*(vel1.w+vel2.w);
    Z.w=dis2.w;
    %flexible wheelset
    
    %update the contact force
    switch contactID
        %-----Hertz spring----
        case 5
            
            %nonlinear
            penetration = Z.w-Z.r-Z.irr;
            if penetration > 0
                penetration=0;
            end
            
            
            F = inp.mater(contactID).Data*(abs(penetration).^1.5);
        case 10
            %linear
            penetration = Z.w-Z.r-Z.irr;
            if penetration > 0
                penetration=0;
            end
            %             F = inp.mater(contactID).Data*abs(penetration);
            %
            %         case 8
            %------Winkler bedding----
            F=winkler_bedding(15, X_w_t, dis2.r,dis2.w,Z_global.irr, geo, mat_trk,inp,-1);
            
            
            
    end
    
    ite=ite+1;
    %     scatter(ite,F-F0);
    
    %check if the
    if abs(F-F0) <= etol
        display(['Convergence reached in ', num2str(ite), ' iterations'])
        break
    end
    
    
    
    if ite>=100
        display(['Convergence not reached within 100 iterations.Out of balance force is ', num2str(F-F0)])
        break
    end
end
end

