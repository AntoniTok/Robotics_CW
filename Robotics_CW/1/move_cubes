% Read the position of the dynamixel horn with the torque off
% The code executes for a given amount of time then terminates


clc;
clear all;

lib_name = '';

if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

%% ---- Control Table Addresses ---- %%

ADDR_PRO_TORQUE_ENABLE       = 64;           % Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION       = 116; 
ADDR_PRO_PRESENT_POSITION    = 132; 
ADDR_PRO_OPERATING_MODE      = 11;
PROFILE_VELOCITY = 112;

%% ---- Other Settings ---- %%

% Protocol version
PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

% Default setting
%for the arms chage later once we know the ids
ID_BASE = 11;
ID_SHOULDER =12 ;
ID_ELBOW = 13;
ID_WRIST =14 ;
ID_GRIPPER = 15;
DXL_ID1 = 12; %delet later
DXL_ID2 = 11; %delete later

BAUDRATE                    = 1000000;
DEVICENAME                  = 'COM5';       % Check which port is being used on your controller
                                            % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'
                                            
TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = -150000;      % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 150000;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

%% ------------------ %%

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);



% Initialize PacketHandler Structs
packetHandler();

index = 1;
dxl_comm_result = COMM_TX_FAIL;           % Communication result
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE DXL_MAXIMUM_POSITION_VALUE];         % Goal position

dxl_error = 0;                              % Dynamixel error
dxl_present_position = 0;                   % Present position


% Open port
if (openPort(port_num))
    fprintf('Port Open\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port\n');
    input('Press any key to terminate...\n');
    return;
end


% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Baudrate Set\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

% ----- SET MOTION LIMITS ----------- %
ADDR_MAX_POS = 48;
ADDR_MIN_POS = 52;
MAX_POS = 3400;
MIN_POS = 600;
% Set max position limit
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_MAX_POS, MAX_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_MAX_POS, MAX_POS);
% Set min position limit
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_MIN_POS, MIN_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_MIN_POS, MIN_POS);
% ---------------------------------- %


%write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, 8, 0);

%BAUDRATE = 9600;

% Put actuator into Position Control Mode
%write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_OPERATING_MODE, 3);


% Disable Dynamixel Torque
%write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
%write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, 1);

write4ByteTxRx(port_num, PROTOCOL_VERSION, ID_BASE, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write4ByteTxRx(port_num, PROTOCOL_VERSION, ID_WRIST, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write4ByteTxRx(port_num, PROTOCOL_VERSION, ID_ELBOW, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write4ByteTxRx(port_num, PROTOCOL_VERSION, ID_SHOULDER, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write4ByteTxRx(port_num, PROTOCOL_VERSION, ID_GRIPPER, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);

dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);

if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION,dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end


% test move 1
write4ByteTxRx(port_num, PROTOCOL_VERSION, ID_BASE, ADDR_PRO_GOAL_POSITION, 2048);



function moveToXYZ(port_num, x, y, z)

    [t1, t2, t3, t4] = Inverse_kinematics(x, y, z);

    write4ByteTxRx(port_num,PROTOCOL_VERSION, ID_BASE,    ADDR_PRO_GOAL_POSITION , deg2tick(t1));
    write4ByteTxRx(port_num, PROTOCOL_VERSION, ID_SHOULDER, ADDR_PRO_GOAL_POSITION , deg2tick(t2));
    write4ByteTxRx(port_num, PROTOCOL_VERSION, ID_ELBOW,   ADDR_PRO_GOAL_POSITION , deg2tick(t3));
    write4ByteTxRx(port_num, PROTOCOL_VERSION, ID_WRIST,   ADDR_PRO_GOAL_POSITION , deg2tick(t4));

    pause(0.4);
end


Z_ABOVE = 100;
Z_PICK  = 40;

function pickCube(port_num, x, y)
    moveToXYZ(port_num, x, y, Z_ABOVE);
    moveToXYZ(port_num, x, y, Z_PICK);
    closeGripper(port_num); %need to do this
    moveToXYZ(port_num, x, y, Z_ABOVE);
end

function placeCube(port_num, x, y)
    moveToXYZ(port_num, x, y, Z_ABOVE);
    moveToXYZ(port_num, x, y, Z_PICK);
    openGripper(port_num); %need to do this 
    moveToXYZ(port_num, x, y, Z_ABOVE);
end

% Disable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Close port
closePort(port_num);
fprintf('Port Closed \n');

% Unload Library
unloadlibrary(lib_name);

close all;
clear all;
