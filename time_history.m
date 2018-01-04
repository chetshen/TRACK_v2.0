 function [dis_out,vel_out,acc_out]=time_history(nodeNumber, dis, vel, acc, activeDof, dofID)
if nargin < 6
    dofID=1;
end
dof=2*(nodeNumber-1)+dofID;
index=ismember(activeDof,dof);


dis_out=dis(:,index);
vel_out=vel(:,index);
acc_out=acc(:,index);
 end
 