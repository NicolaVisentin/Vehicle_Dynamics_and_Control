function A=ComputeA(u,par)

m=par.mass;
Caf=par.Calpha_front;
Car=par.Calpha_rear;
lf=par.l_f;
lr=par.l_r;
Iz=par.Izz;

A=zeros(2,2);
A(1,1)=-(Caf+Car)/(m*u);
A(1,2)=(-lf*Caf+lr*Car)/(m*u)-u;
A(2,1)=(lr*Car-lf*Caf)/(Iz*u);
A(2,2)=-(lf^2*Caf+lr^2*Car)/(Iz*u);

end