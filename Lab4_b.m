Tm=0.63;
k_m=1/36;
k0=0.24;
kt=0.0034;
km=237.2;
V_7805 = 5.39;
Vref_arduino = 5.064;
% x1 -> velocity (ω) , x2 -> position (θ)
% Matrix initialization
positionData=[];
velocityData=[];
timeData=[];
positionEst=[];
velocityEst=[];
controllerData=[];
% State Feedback Gains
% Desired poles
p_est=17;
p1=2*p_est;
p2=p_est^2;
p_control=5.8;
% L matrix values
l2=p1-(1/Tm);
l1=(p2-(l2/Tm))*kt/(k_m*k0);
% gain k1 for x1,k2 for x2
k1=(2*Tm*p_control-1)/(km*kt)-2.8;  %%or k1=5 kai k2=13.8
k2=(Tm*p_control^2)/(km*k_m*k0)+0.4;
kr=k2;

% Output signal to stop motor
analogWrite(a, 6, 0);
analogWrite(a, 9, 0);

% Initial estimate for x1,x2 
x1_est=0;
x2_est=0;
y_r=5;
u=-k1*x1_est-k2*x2_est+kr*y_r;

% Set time
t=0;

% START CLOCK
tic
 
while(t<5)  
   
	velocity = analogRead(a, 3);
	position = analogRead(a, 5);
    
	theta = 3 * Vref_arduino * position / 1023;
	vtacho = 2 * (2 * velocity * Vref_arduino / 1023 - V_7805);
    x1=vtacho;
    x2=theta;
    % Output signal for state feedback
    x1dot_est=-x1_est/Tm +(km*kt*u)/Tm+l1*(theta-x2_est);
    x2dot_est=k_m*k0*x1_est/kt+l2*(theta-x2_est);
    dt=toc-t;
    t=toc;
    x1_est=x1_est+dt*x1dot_est;
    x2_est=x2_est+dt*x2dot_est;
    u=-k1*x1_est-k2*x2_est+k2*y_r;
    % Output signal for state feedback
    if abs(u) > 255
		u = sign(u) * 255;
    end
    if u > 0
		analogWrite(a, 6, 0);
		analogWrite(a, 9, min(round(u / 2 * 255 / Vref_arduino), 255));
    else
		analogWrite(a, 9, 0);
		analogWrite(a, 6, min(round(-u / 2 * 255 / Vref_arduino), 255));
    end

   

	% Update matrices with information    
	timeData = [timeData t];
	positionData = [positionData theta];
	velocityData = [velocityData vtacho];
    positionEst = [positionEst x2_est];
    velocityEst= [velocityEst x1_est];
    controllerData=[controllerData u];

end

for(i=1:length(timeData))

    y_r(i)=5;
end



% OUTPUT ZERO CONTROL SIGNAL TO STOP MOTOR  %
analogWrite(a, 6, 0);
analogWrite(a, 9, 0);
%%

figure(1)
plot(timeData, positionData);
title('position')

figure(2)
plot(timeData, velocityData);
title('velocity')

figure(3)
plot(timeData,velocityData,timeData,velocityEst,'p--');
title('real and estimated velocity')

figure(4)
plot(timeData,positionData,timeData,positionEst,'p--',timeData,y_r);
title('real and estimated position')

figure(5)
plot(timeData,controllerData);
title('controller')

