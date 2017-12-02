-- This is a very simple EXAMPLE navigation program, which avoids obstacles using the Braitenberg algorithm
function subscriber_callback(msg)
    -- This is the subscriber callback function
    simAddStatusbarMessage('subscriber receiver: ' .. msg.linear.x)
    simAuxiliaryConsolePrint(console_debug,'linear: ' .. msg.linear.x)
    simAuxiliaryConsolePrint(console_debug,' angular: ' .. msg.angular.z .. '\n')
    vLeft = 5*msg.linear.x - 2*msg.angular.z
    vRight = 5*msg.linear.x + 2*msg.angular.z
    simSetJointTargetVelocity(motorLeft,vLeft)
    simSetJointTargetVelocity(motorRight,vRight)
end

if (sim_call_type==sim_childscriptcall_initialization) then 
    usensors={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}
    for i=1,16,1 do
        usensors[i]=simGetObjectHandle("Pioneer_p3dx_ultrasonicSensor"..i)
    end
    motorLeft=simGetObjectHandle("Pioneer_p3dx_leftMotor")
    motorRight=simGetObjectHandle("Pioneer_p3dx_rightMotor")
    noDetectionDist=0.5
    maxDetectionDist=0.2
    detect={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
    braitenbergL={-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
    braitenbergR={-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
    v0=2
    
    -- create the console to print some debug infomation
    console_debug=simAuxiliaryConsoleOpen('debug info',100000,1)
    -- Check the required RosInterface:
    moduleName=0
    index=0
    rosInterfacePresent=false
    while moduleName do
        moduleName=simGetModuleName(index)
        if (moduleName=='RosInterface') then
            rosInterfacePresent=true
        end
        index=index+1
    end

    if rosInterfacePresent then   
        subscriber = simExtRosInterface_subscribe('/key_vel','geometry_msgs/Twist','subscriber_callback')
    else
        simAuxiliaryConsolePrint(console_debug,'rosinterface has not been loaded\n -- pause...')
        simPauseSimulation()
    end
end 

if (sim_call_type==sim_childscriptcall_cleanup) then 
-- Following not really needed in a simulation script (i.e. automatically shut down at simulation end):
    if rosInterfacePresent then
        simExtRosInterface_shutdownSubscriber(subscriber)
    end
end 

if (sim_call_type==sim_childscriptcall_actuation) then 
    --[[
    for i=1,16,1 do
        res,dist=simReadProximitySensor(usensors[i])
        if (res>0) and (dist<noDetectionDist) then
            if (dist<maxDetectionDist) then
                dist=maxDetectionDist
            end
            detect[i]=1-((dist-maxDetectionDist)/(noDetectionDist-maxDetectionDist))
        else
            detect[i]=0
        end
    end
    
    vLeft=v0
    vRight=v0
    
    for i=1,16,1 do
        vLeft=vLeft+braitenbergL[i]*detect[i]
        vRight=vRight+braitenbergR[i]*detect[i]
    end
    
    simSetJointTargetVelocity(motorLeft,vLeft)
    simSetJointTargetVelocity(motorRight,vRight)
    ]]--
end 
