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
eData=[];
timeData=[];
zData=[];
uData=[];
% State Feedback Gains
% Desired pole for gains k1,k2 calculation;
p=2;
m=3.7;
%gains for k1,k2,k3(z)
k1=((2*p+m)*Tm-1)/(km*kt);
k2=(p^2+2*p*m)*Tm/(km*k_m*k0);
k3=m*p^2*Tm/(km*k_m*k0); %% or k1=3,k2=6 kai k3=6;

%%STOP MOTOR
analogWrite(a, 6, 0);
analogWrite(a, 9, 0);

% Desired position
y_r=5;
% Initial value of z
z=0;

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
    zdot=x2-y_r;
    dt=toc-t;
    t=toc;
    z=z+zdot*dt;
    %ERROR
    e=y_r-x2;
    % Output signal for dynamic state feedback
    u=-k1*x1-k2*x2-k3*z;
    
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
	eData = [eData e];
    zData=[zData z];
    uData=[uData u];
      
end
for(i=1:length(timeData))

    y_r(i)=5;
end


% OUTPUT ZERO CONTROL SIGNAL TO STOP MOTOR  %
analogWrite(a, 6, 0);
analogWrite(a, 9, 0);


figure(1)
plot(timeData, positionData);
title('position')

%%figure(2)
plot(timeData, velocityData);
title('velocity')

figure(3)
plot(timeData,positionData,timeData,y_r);
title('current and desired position')

figure(4)
plot(timeData,uData);
title('controller')
