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
% State Feedback Gains
% Desired poles ranging 30-40-50-70
p=50;
p1=2*p;
p2=p*p;
% L matrix values
l2=p1-(1/Tm);
l1=((p2-(l2/Tm))*kt)/(k_m*k0);
u=7;
% Output signal to stop motor
analogWrite(a, 6, 0);
analogWrite(a, 9, 0);

% Initial estimate for x1,x2 
x1_est=0;
x2_est=0;

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
    x1dot_est=-x1_est/Tm+km*kt*u/Tm+l1*(x2-x2_est);
    x2dot_est=k_m*k0*x1_est/kt+l2*(x2-x2_est);
    dt=toc-t;
    t=toc;
    x1_est=x1_est+x1dot_est*dt;
    x2_est=x2_est+dt*x2dot_est;
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

    % Set time to current time
	

	% Update matrices with information    
	timeData = [timeData t];
	positionData = [positionData theta];
	velocityData = [velocityData vtacho];
    positionEst = [positionEst x2_est];
    velocityEst= [velocityEst x1_est];

end



% OUTPUT ZERO CONTROL SIGNAL TO STOP MOTOR  %
analogWrite(a, 6, 0);
analogWrite(a, 9, 0);


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
plot(timeData,positionData,timeData,positionEst,'p--');
title('real and estimated position')



