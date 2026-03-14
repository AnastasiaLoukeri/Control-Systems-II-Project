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
uData=[];
% State Feedback Gains
% Desired pole for gains k1,k2 calculation;
p=5.8;

% gain k1 for x1,k2 for x2
k1=(2*Tm*p-1)/(km*kt);  
k2=(Tm*p^2)/(km*k_m*k0);
kr=k2;

% Output signal to stop motor
analogWrite(a, 6, 0);
analogWrite(a, 9, 0);

% Desired position
y_r=5;

% Set time
t=0;

% WAIT A KEY TO PROCEED

% START CLOCK
tic
 
while(t<5)  
   
	velocity = analogRead(a, 3);
	position = analogRead(a, 5);

	theta = 3 * Vref_arduino * position / 1023;

	vtacho = 2 * (2 * velocity * Vref_arduino / 1023 - V_7805);
    x1=vtacho;
    x2=theta;
    % ERROR
    e=y_r-theta;
    % Output signal for state feedback
    u=-k1*x1+k2*e;
    
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
	t = toc;

	% Update matrices with information    
	timeData = [timeData t];
	positionData = [positionData theta];
	velocityData = [velocityData vtacho];
	eData = [eData e];
    uData = [uData u];

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

figure(2)
plot(timeData, velocityData);
title('velocity')

figure(3)
plot(timeData,positionData,timeData,y_r);
title('current and desired position')








