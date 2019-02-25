% 
%
%
%   
%  


function quadcopter_dynamic_model_block(block)
    
setup(block);
  
% SETUP BLOCK

function setup(block)

    %Input ports
    
    block.NumInputPorts=7; % 5 Inputs
     
    block.InputPort(1).Dimensions=1; % Force x
    block.InputPort(2).Dimensions=1; % Torque_x
    block.InputPort(3).Dimensions=1; % Torque_y
    block.InputPort(4).Dimensions=1; % Torque_z
    block.InputPort(5).Dimensions=1; % Omega_r
    block.InputPort(6).Dimensions=1; % f_x wind
    block.InputPort(7).Dimensions=1; % f_y wind
    
    block.InputPort(1).DirectFeedthrough=false; % elv false, de ha gond van, akk try true
    block.InputPort(2).DirectFeedthrough=false; 
    block.InputPort(3).DirectFeedthrough=false;
    block.InputPort(4).DirectFeedthrough=false;
    block.InputPort(5).DirectFeedthrough=false;
   
    
    %Output ports
    
    block.NumOutputPorts=12; 

    block.OutputPort(1).Dimensions=1; % x
    block.OutputPort(2).Dimensions=1; % d_x
    block.OutputPort(3).Dimensions=1; % y
    block.OutputPort(4).Dimensions=1; % d_y
    block.OutputPort(5).Dimensions=1; % z
    block.OutputPort(6).Dimensions=1; % d_z
    block.OutputPort(7).Dimensions=1; % fi - Roll
    block.OutputPort(8).Dimensions=1; % d_fi
    block.OutputPort(9).Dimensions=1; % theta - Pitch
    block.OutputPort(10).Dimensions=1; % d_theta
    block.OutputPort(11).Dimensions=1; % pszi - Yaw
    block.OutputPort(12).Dimensions=1; % d_pszi
    
    % Continuous states
    
    block.NumContStates=12;
    
    % Dialog parameters
    
    block.NumDialogPrms=12; % g, m, I_x, I_y, I_z, I_r, Kt_x, Kt_y, Kt_z, Kr_x, Kr_y, Kr_z
    
    block.DialogPrmsTunable={'Tunable','Tunable','Tunable','Tunable','Tunable','Tunable','Tunable','Tunable','Tunable','Tunable','Tunable','Tunable'}; % All parameters are tunable
    
    % Sampling time
    
    block.SampleTimes=[0 0]; % The system is continuous thus sampling time is set to [0 0]
    
    % Registration of the used functions
        
    block.RegBlockMethod('InitializeConditions',@InitCon); % Init
    block.RegBlockMethod('Outputs',@Outputs); % Calculate outputs
    block.RegBlockMethod('Derivatives',@Derivatives); % Calculate derivatives
    
    % In case of using more outputs than one
    block.RegBlockMethod('SetInputPortSamplingMode',@SetInput);

function InitCon(block)

    % Initial values of the continuous states
    %block.ContStates.Data=[0 0 0 0 0 0 0.001 0.001 0.001 0 0 0]';
    block.ContStates.Data=[0 0 0 0 0 0 0 0 0 0 0 0]';
    
    
function Outputs(block)

    % Calculating outputs from continuous states
    
    block.OutputPort(1).Data=block.ContStates.Data(1); % x
    block.OutputPort(2).Data=block.ContStates.Data(4); % d_x
    block.OutputPort(3).Data=block.ContStates.Data(2); % y
    block.OutputPort(4).Data=block.ContStates.Data(5); % d_y
    block.OutputPort(5).Data=block.ContStates.Data(3); % z
    block.OutputPort(6).Data=block.ContStates.Data(6); % d_z
    block.OutputPort(7).Data=block.ContStates.Data(7); % fi
    block.OutputPort(8).Data=block.ContStates.Data(10); % d_fi
    block.OutputPort(9).Data=block.ContStates.Data(8); % theta
    block.OutputPort(10).Data=block.ContStates.Data(11); % d_theta
    block.OutputPort(11).Data=block.ContStates.Data(9); % pszi
    block.OutputPort(12).Data=block.ContStates.Data(12); % d_pszi

function Derivatives(block)

    % Calculate Derivatives
    
    % 1. Parameters value
    g=block.DialogPrm(1).Data; %gravitational acc. [m/s^2]
    m=block.DialogPrm(2).Data; %weight [kg]
    I_x=block.DialogPrm(3).Data; % inertia - x [kg*m^2]
    I_y=block.DialogPrm(4).Data; % inertia - y [kg*m^2]
    I_z=block.DialogPrm(5).Data; % inertia - z [kg*m^2]
    I_r=block.DialogPrm(6).Data; % inertia - z [kg*m^2]
    kt_x=block.DialogPrm(7).Data; % drag force coefficient - x [Ns/m]
    kt_y=block.DialogPrm(8).Data; % drag force coefficient - y [Ns/m]
    kt_z=block.DialogPrm(9).Data; % drag force coefficient - z [Ns/m]
    kr_x=block.DialogPrm(10).Data; % drag torque coefficient [Nms]
    kr_y=block.DialogPrm(11).Data; % drag torque coefficient [Nms]
    kr_z=block.DialogPrm(12).Data; % drag torque coefficient [Nms]
       
    % Az alábbi változókat csak a deriváltak egyszerûbb,érthetõbb
    % és szemléletesebb leírása kedvéért hozzuk létre
    
    x=block.ContStates.Data(1); % x
    y=block.ContStates.Data(2); % y
    z=block.ContStates.Data(3); % z
    d_x=block.ContStates.Data(4); % derivative of x
    d_y=block.ContStates.Data(5); % derivative of y
    d_z=block.ContStates.Data(6); % derivative of z
    fi=block.ContStates.Data(7); % fi
    theta=block.ContStates.Data(8); % theta
    pszi=block.ContStates.Data(9); % pszi
    d_fi=block.ContStates.Data(10); % derivative of fi
    d_theta=block.ContStates.Data(11); % derivative of theta
    d_pszi=block.ContStates.Data(12); % derivative of pszi

    f=block.InputPort(1).Data; % Force_x = F_1+F_2+F_3+F_4
    T_x=block.InputPort(2).Data; % Torque_x
    T_y=block.InputPort(3).Data; % Torque_y
    T_z=block.InputPort(4).Data; % Torque_z
    Omega_r=block.InputPort(5).Data; % Omega_r = Omega_1+Omega_2+Omega_3+Omega_4
    f_x=block.InputPort(6).Data; % wind_x
    f_y=block.InputPort(7).Data; % wind_y
%     if ((fi==0 && theta==0) || (pszi==0 && theta==0))
%         dd_x=f_x;
%         dd_y=f_y;
%     else
      %dd_x=(d_y*(kt_y*(cos(fi)*cos(pszi) + sin(fi)*sin(pszi)*sin(theta))*(cos(fi)*sin(pszi) - cos(pszi)*sin(fi)*sin(theta)) + kt_z*(sin(fi)*sin(pszi) + cos(fi)*cos(pszi)*sin(theta))*(cos(pszi)*sin(fi) - cos(fi)*sin(pszi)*sin(theta)) - kt_x*cos(pszi)*cos(theta)^2*sin(pszi)))/m - (d_x*(kt_y*(cos(fi)*sin(pszi) - cos(pszi)*sin(fi)*sin(theta))^2 + kt_z*(sin(fi)*sin(pszi) + cos(fi)*cos(pszi)*sin(theta))^2 + kt_x*cos(pszi)^2*cos(theta)^2))/m + (d_z*(kt_x*cos(pszi)*cos(theta)*sin(theta) - kt_z*cos(fi)*cos(theta)*(sin(fi)*sin(pszi) + cos(fi)*cos(pszi)*sin(theta)) + kt_y*cos(theta)*sin(fi)*(cos(fi)*sin(pszi) - cos(pszi)*sin(fi)*sin(theta))))/m+(f*(sin(fi)*sin(pszi) + cos(fi)*cos(pszi)*sin(theta)))/m;
     %  dd_y=(d_z*(kt_x*cos(theta)*sin(pszi)*sin(theta) + kt_z*cos(fi)*cos(theta)*(cos(pszi)*sin(fi) - cos(fi)*sin(pszi)*sin(theta)) - kt_y*cos(theta)*sin(fi)*(cos(fi)*cos(pszi) + sin(fi)*sin(pszi)*sin(theta))))/m - (d_y*(kt_y*(cos(fi)*cos(pszi) + sin(fi)*sin(pszi)*sin(theta))^2 + kt_z*(cos(pszi)*sin(fi) - cos(fi)*sin(pszi)*sin(theta))^2 + kt_x*cos(theta)^2*sin(pszi)^2))/m + (d_x*(kt_y*(cos(fi)*cos(pszi) + sin(fi)*sin(pszi)*sin(theta))*(cos(fi)*sin(pszi) - cos(pszi)*sin(fi)*sin(theta)) + kt_z*(sin(fi)*sin(pszi) + cos(fi)*cos(pszi)*sin(theta))*(cos(pszi)*sin(fi) - cos(fi)*sin(pszi)*sin(theta)) - kt_x*cos(pszi)*cos(theta)^2*sin(pszi)))/m-(f*(cos(pszi)*sin(fi) - cos(fi)*sin(pszi)*sin(theta)))/m;
%     end
    
    
    
    % Calculate derivatives
    block.Derivatives.Data=[d_x;                % 1st derivative of x
                            d_y;                % 1st derivative of y
                            d_z;                % 1st derivative of z
                            (d_y*(kt_y*(cos(fi)*cos(pszi) + sin(fi)*sin(pszi)*sin(theta))*(cos(fi)*sin(pszi) - cos(pszi)*sin(fi)*sin(theta)) + kt_z*(sin(fi)*sin(pszi) + cos(fi)*cos(pszi)*sin(theta))*(cos(pszi)*sin(fi) - cos(fi)*sin(pszi)*sin(theta)) - kt_x*cos(pszi)*cos(theta)^2*sin(pszi)))/m - (d_x*(kt_y*(cos(fi)*sin(pszi) - cos(pszi)*sin(fi)*sin(theta))^2 + kt_z*(sin(fi)*sin(pszi) + cos(fi)*cos(pszi)*sin(theta))^2 + kt_x*cos(pszi)^2*cos(theta)^2))/m + (d_z*(kt_x*cos(pszi)*cos(theta)*sin(theta) - kt_z*cos(fi)*cos(theta)*(sin(fi)*sin(pszi) + cos(fi)*cos(pszi)*sin(theta)) + kt_y*cos(theta)*sin(fi)*(cos(fi)*sin(pszi) - cos(pszi)*sin(fi)*sin(theta))))/m+(f*(sin(fi)*sin(pszi) + cos(fi)*cos(pszi)*sin(theta))+f_x)/m;
                            %dd_x;
                           %(d_y*(kt_y*(cos(fi)*cos(pszi) + sin(fi)*sin(pszi)*sin(theta))*(cos(fi)*sin(pszi) - cos(pszi)*sin(fi)*sin(theta)) + kt_z*(sin(fi)*sin(pszi) + cos(fi)*cos(pszi)*sin(theta))*(cos(pszi)*sin(fi) - cos(fi)*sin(pszi)*sin(theta)) - kt_x*cos(pszi)*cos(theta)^2*sin(pszi)))/m - (d_x*(kt_y*(cos(fi)*sin(pszi) - cos(pszi)*sin(fi)*sin(theta))^2 + kt_z*(sin(fi)*sin(pszi) + cos(fi)*cos(pszi)*sin(theta))^2 + kt_x*cos(pszi)^2*cos(theta)^2))/m + (d_z*(kt_x*cos(pszi)*cos(theta)*sin(theta) - kt_z*cos(fi)*cos(theta)*(sin(fi)*sin(pszi) + cos(fi)*cos(pszi)*sin(theta)) + kt_y*cos(theta)*sin(fi)*(cos(fi)*sin(pszi) - cos(pszi)*sin(fi)*sin(theta))))/m+(f_x*(sin(fi)*sin(pszi) + cos(fi)*cos(pszi)*sin(theta)))/m; % 2nd derivative of x
                           (d_z*(kt_x*cos(theta)*sin(pszi)*sin(theta) + kt_z*cos(fi)*cos(theta)*(cos(pszi)*sin(fi) - cos(fi)*sin(pszi)*sin(theta)) - kt_y*cos(theta)*sin(fi)*(cos(fi)*cos(pszi) + sin(fi)*sin(pszi)*sin(theta))))/m - (d_y*(kt_y*(cos(fi)*cos(pszi) + sin(fi)*sin(pszi)*sin(theta))^2 + kt_z*(cos(pszi)*sin(fi) - cos(fi)*sin(pszi)*sin(theta))^2 + kt_x*cos(theta)^2*sin(pszi)^2))/m + (d_x*(kt_y*(cos(fi)*cos(pszi) + sin(fi)*sin(pszi)*sin(theta))*(cos(fi)*sin(pszi) - cos(pszi)*sin(fi)*sin(theta)) + kt_z*(sin(fi)*sin(pszi) + cos(fi)*cos(pszi)*sin(theta))*(cos(pszi)*sin(fi) - cos(fi)*sin(pszi)*sin(theta)) - kt_x*cos(pszi)*cos(theta)^2*sin(pszi)))/m-(f*(cos(pszi)*sin(fi) - cos(fi)*sin(pszi)*sin(theta))+f_y)/m;
                           %dd_y;
                           % (d_z*(kt_x*cos(theta)*sin(pszi)*sin(theta) + kt_z*cos(fi)*cos(theta)*(cos(pszi)*sin(fi) - cos(fi)*sin(pszi)*sin(theta)) - kt_y*cos(theta)*sin(fi)*(cos(fi)*cos(pszi) + sin(fi)*sin(pszi)*sin(theta))))/m - (d_y*(kt_y*(cos(fi)*cos(pszi) + sin(fi)*sin(pszi)*sin(theta))^2 + kt_z*(cos(pszi)*sin(fi) - cos(fi)*sin(pszi)*sin(theta))^2 + kt_x*cos(theta)^2*sin(pszi)^2))/m + (d_x*(kt_y*(cos(fi)*cos(pszi) + sin(fi)*sin(pszi)*sin(theta))*(cos(fi)*sin(pszi) - cos(pszi)*sin(fi)*sin(theta)) + kt_z*(sin(fi)*sin(pszi) + cos(fi)*cos(pszi)*sin(theta))*(cos(pszi)*sin(fi) - cos(fi)*sin(pszi)*sin(theta)) - kt_x*cos(pszi)*cos(theta)^2*sin(pszi)))/m-(f_y*(cos(pszi)*sin(fi) - cos(fi)*sin(pszi)*sin(theta)))/m; % 2nd derivative of y
                           (d_y*(kt_x*cos(theta)*sin(pszi)*sin(theta) + kt_z*cos(fi)*cos(theta)*(cos(pszi)*sin(fi) - cos(fi)*sin(pszi)*sin(theta)) - kt_y*cos(theta)*sin(fi)*(cos(fi)*cos(pszi) + sin(fi)*sin(pszi)*sin(theta))))/m - g - (d_z*(kt_z*cos(fi)^2*cos(theta)^2 + kt_y*cos(theta)^2*sin(fi)^2 + kt_x*sin(theta)^2))/m + (d_x*(kt_x*cos(pszi)*cos(theta)*sin(theta) - kt_z*cos(fi)*cos(theta)*(sin(fi)*sin(pszi) + cos(fi)*cos(pszi)*sin(theta)) + kt_y*cos(theta)*sin(fi)*(cos(fi)*sin(pszi) - cos(pszi)*sin(fi)*sin(theta))))/m+ (f*cos(fi)*cos(theta))/m; % 2nd derivative of z
                            d_fi;               % 1st derivative of fi     
                            d_theta;            % 1st derivative of theta
                            d_pszi;             % 1st derivative of pszi
                            (sin(fi)*sin(theta)*((I_x*d_fi - I_x*d_pszi*sin(theta))*(d_theta*sin(fi) - d_pszi*cos(fi)*cos(theta)) + (d_fi - d_pszi*sin(theta))*(I_r*Omega_r - I_z*d_theta*sin(fi) + I_z*d_pszi*cos(fi)*cos(theta)) - I_y*d_pszi*(d_fi*cos(fi)*cos(theta) - d_theta*sin(fi)*sin(theta)) - d_theta*kr_y*cos(fi) + I_y*d_fi*d_theta*sin(fi) - d_pszi*kr_y*cos(theta)*sin(fi)))/(I_y*cos(theta)*cos(fi)^2 + I_y*cos(theta)*sin(fi)^2) - (d_fi*kr_x + (d_theta*cos(fi) + d_pszi*cos(theta)*sin(fi))*(I_r*Omega_r - I_z*d_theta*sin(fi) + I_z*d_pszi*cos(fi)*cos(theta)) + (d_theta*sin(fi) - d_pszi*cos(fi)*cos(theta))*(I_y*d_theta*cos(fi) + I_y*d_pszi*cos(theta)*sin(fi)) - d_pszi*kr_x*sin(theta) - I_x*d_pszi*d_theta*cos(theta))/I_x + (cos(fi)*sin(theta)*((I_x*d_fi - I_x*d_pszi*sin(theta))*(d_theta*cos(fi) + d_pszi*cos(theta)*sin(fi)) - (d_fi - d_pszi*sin(theta))*(I_y*d_theta*cos(fi) + I_y*d_pszi*cos(theta)*sin(fi)) + d_theta*kr_z*sin(fi) + I_z*d_pszi*(d_fi*cos(theta)*sin(fi) + d_theta*cos(fi)*sin(theta)) + I_z*d_fi*d_theta*cos(fi) - d_pszi*kr_z*cos(fi)*cos(theta)))/(I_z*cos(theta)*cos(fi)^2 + I_z*cos(theta)*sin(fi)^2)+(T_x/I_x + (T_z*cos(fi)*sin(theta))/(I_z*cos(theta)*cos(fi)^2 + I_z*cos(theta)*sin(fi)^2) + (T_y*sin(fi)*sin(theta))/(I_y*cos(theta)*cos(fi)^2 + I_y*cos(theta)*sin(fi)^2)); % 2nd derivative of fi
                            (cos(fi)*((I_x*d_fi - I_x*d_pszi*sin(theta))*(d_theta*sin(fi) - d_pszi*cos(fi)*cos(theta)) + (d_fi - d_pszi*sin(theta))*(I_r*Omega_r - I_z*d_theta*sin(fi) + I_z*d_pszi*cos(fi)*cos(theta)) - I_y*d_pszi*(d_fi*cos(fi)*cos(theta) - d_theta*sin(fi)*sin(theta)) - d_theta*kr_y*cos(fi) + I_y*d_fi*d_theta*sin(fi) - d_pszi*kr_y*cos(theta)*sin(fi)))/(I_y*cos(fi)^2 + I_y*sin(fi)^2) - (sin(fi)*((I_x*d_fi - I_x*d_pszi*sin(theta))*(d_theta*cos(fi) + d_pszi*cos(theta)*sin(fi)) - (d_fi - d_pszi*sin(theta))*(I_y*d_theta*cos(fi) + I_y*d_pszi*cos(theta)*sin(fi)) + d_theta*kr_z*sin(fi) + I_z*d_pszi*(d_fi*cos(theta)*sin(fi) + d_theta*cos(fi)*sin(theta)) + I_z*d_fi*d_theta*cos(fi) - d_pszi*kr_z*cos(fi)*cos(theta)))/(I_z*cos(fi)^2 + I_z*sin(fi)^2)+((T_y*cos(fi))/(I_y*cos(fi)^2 + I_y*sin(fi)^2) - (T_z*sin(fi))/(I_z*cos(fi)^2 + I_z*sin(fi)^2)); % 2nd derivative of theta
                            (cos(fi)*((I_x*d_fi - I_x*d_pszi*sin(theta))*(d_theta*cos(fi) + d_pszi*cos(theta)*sin(fi)) - (d_fi - d_pszi*sin(theta))*(I_y*d_theta*cos(fi) + I_y*d_pszi*cos(theta)*sin(fi)) + d_theta*kr_z*sin(fi) + I_z*d_pszi*(d_fi*cos(theta)*sin(fi) + d_theta*cos(fi)*sin(theta)) + I_z*d_fi*d_theta*cos(fi) - d_pszi*kr_z*cos(fi)*cos(theta)))/(I_z*cos(theta)*cos(fi)^2 + I_z*cos(theta)*sin(fi)^2) + (sin(fi)*((I_x*d_fi - I_x*d_pszi*sin(theta))*(d_theta*sin(fi) - d_pszi*cos(fi)*cos(theta)) + (d_fi - d_pszi*sin(theta))*(I_r*Omega_r - I_z*d_theta*sin(fi) + I_z*d_pszi*cos(fi)*cos(theta)) - I_y*d_pszi*(d_fi*cos(fi)*cos(theta) - d_theta*sin(fi)*sin(theta)) - d_theta*kr_y*cos(fi) + I_y*d_fi*d_theta*sin(fi) - d_pszi*kr_y*cos(theta)*sin(fi)))/(I_y*cos(theta)*cos(fi)^2 + I_y*cos(theta)*sin(fi)^2)+((T_z*cos(fi))/(I_z*cos(theta)*cos(fi)^2 + I_z*cos(theta)*sin(fi)^2) + (T_y*sin(fi))/(I_y*cos(theta)*cos(fi)^2 + I_y*cos(theta)*sin(fi)^2)); % 2nd derivative of pszi
                            ];                                            
%endfunction

function SetInput(block,idx,fd)

    % Ezt a metódust mindenképpen implementálni kell, ha egynél több
    % kimeneti portot használunk. A metódus gyakorlatilag a Simulinktõl
    % érkezõ kérést (frame data sampling mód beállítás) továbbítja a blokk
    % objektum felé. Az idx paraméter a kérdéses (beállítandó) input port 
    % száma, fd pedig a beállítandó érték (0/1).

    block.InputPort(idx).SamplingMode=fd;
    block.OutputPort(1).SamplingMode=fd;
    block.OutputPort(2).SamplingMode=fd;
    block.OutputPort(3).SamplingMode=fd;
    block.OutputPort(4).SamplingMode=fd;
    block.OutputPort(5).SamplingMode=fd;
    block.OutputPort(6).SamplingMode=fd;
    block.OutputPort(7).SamplingMode=fd;
    block.OutputPort(8).SamplingMode=fd;
    block.OutputPort(9).SamplingMode=fd;
    block.OutputPort(10).SamplingMode=fd;
    block.OutputPort(11).SamplingMode=fd;
    block.OutputPort(12).SamplingMode=fd;
