function sysCall_init() 
    particlesAreVisible=true
    simulateParticles=false
    fakeShadow=true
    
    particleCountPerSecond=430
    particleSize=0.005
    particleDensity=8500
    particleScatteringAngle=30
    particleLifeTime=0.5
    maxParticleCount=50
    
    if simROS then
        --Rotors control
        rotors_ctrl_topic = 'rotors_RPM'
        rotors_ctrl_sub = simROS.subscribe('/'..rotors_ctrl_topic,'std_msgs/Float32MultiArray','rotors_ctrl_cb')
        --Position control
        position_ctrl_topic = 'desired_pos'
        position_ctrl_sub = simROS.subscribe('/'..position_ctrl_topic,'std_msgs/Float32MultiArray','desired_pos_ctrl_cb')
        
        --Velocity publisher 
        vel_pub=simROS.advertise('/drone_velocity', 'std_msgs/Float32MultiArray')
        simROS.publisherTreatUInt8ArrayAsString(vel_pub) -- treat uint8 arrays as strings (much faster, tables/arrays are kind of slow in Lua)
    end
    

    -- Detatch the manipulation sphere:
    targetObj=sim.getObjectHandle('Quadcopter_target#0')
    sim.setObjectParent(targetObj,-1,true)

    -- This control algo was quickly written and is dirty and not optimal. It just serves as a SIMPLE example
    d=sim.getObjectHandle('Quadcopter_base#0')

    propellerHandles={}
    jointHandles={}
    particleObjects={-1,-1,-1,-1}
    local ttype=sim.particle_roughspheres+sim.particle_cyclic+sim.particle_respondable1to4+sim.particle_respondable5to8+sim.particle_ignoresgravity
    if not particlesAreVisible then
        ttype=ttype+sim.particle_invisible
    end
    for i=1,4,1 do
        propellerHandles[i]=sim.getObjectHandle('Quadcopter_propeller_respondable'..i)
        jointHandles[i]=sim.getObjectHandle('Quadcopter_propeller_joint'..i)
        if simulateParticles then
            particleObjects[i]=sim.addParticleObject(ttype,particleSize,particleDensity,{2,1,0.2,3,0.4},particleLifeTime,maxParticleCount,{0.3,0.7,1})
        end
    end
    heli=sim.getObjectHandle(sim.handle_self)

    pParam=2
    iParam=0
    dParam=0
    vParam=-2

    cumul=0
    lastE=0
    pAlphaE=0
    pBetaE=0
    psp2=0
    psp1=0

    prevEuler=0
    cumulA=0
    cumulB=0

    if (fakeShadow) then
        shadowCont=sim.addDrawingObject(sim.drawing_discpoints+sim.drawing_cyclic+sim.drawing_25percenttransparency+sim.drawing_50percenttransparency+sim.drawing_itemsizes,0.2,0,-1,1)
    end
end

function sysCall_cleanup() 
    sim.removeDrawingObject(shadowCont)
    for i=1,#particleObjects,1 do
        sim.removeParticleObject(particleObjects[i])
    end
end 


CONTROL_FROM_ROS = false 
first_ROS_ctrl = false 
vel={0,0,0,0}
function sysCall_actuation() 
    pos=sim.getObjectPosition(d,-1)
    if (fakeShadow) then
        itemData={pos[1],pos[2],0.002,0,0,1,0.2}
        sim.addDrawingObjectItem(shadowCont,itemData)
    end
    quat = sim.getObjectQuaternion(d,-1)
     if simROS then
        --Position Transformation
        pos_transform_msg={
            header={
                stamp=simROS.getTime(),
                frame_id="/world"
            },
            child_frame_id="/drone_0_pos",
            transform={
                -- ROS has definition x=front y=side z=up
                translation={x=pos[1],y=pos[2],z=pos[3]},--V-rep
                rotation={x=quat[1],y=quat[2],z=quat[3],w=quat[4]}--v-rep
            }
        }
        simROS.sendTransform(pos_transform_msg)
        error_euler=sim.getObjectOrientation(d,targetObj)
        --Velocity Transformation
        l=sim.getVelocity(heli)
    
        pos_transform_msg={
            header={
                stamp=simROS.getTime(),
                frame_id="/world"
            },
            child_frame_id="/drone_0_vel",
            transform={
                -- ROS has definition x=front y=side z=up
                translation={x=l[1],y=l[2],z=l[3]},--V-rep
                rotation={x = error_euler[1] , y = error_euler[2] , z = error_euler[3] , w = 1 }--v-rep
            }
        }
        simROS.sendTransform(pos_transform_msg)
        

        
    else
        sim.addLog(sim.verbosity_scripterrors,"ROS interface was not found. Cannot run.")
    end
    if (CONTROL_FROM_ROS == false) then
        -- Vertical control:
        targetPos=sim.getObjectPosition(targetObj,-1)
        pos=sim.getObjectPosition(d,-1)
        l=sim.getVelocity(heli)
        e=(targetPos[3]-pos[3])
        cumul=cumul+e
        pv=pParam*e
        thrust=5.45+pv+iParam*cumul+dParam*(e-lastE)+l[3]*vParam
        lastE=e
        
        -- Horizontal control: 
        sp=sim.getObjectPosition(targetObj,d)
        
        m=sim.getObjectMatrix(d,-1)
        vx={1,0,0}
        vx=sim.multiplyVector(m,vx)
        vy={0,1,0}

        -- print("Rotation Matrix")
        -- print(m[1],"  ",m[2],"  ",m[3],"  ",m[4],"  ")
        -- print(m[5],"  ",m[6],"  ",m[7],"  ",m[8],"  ")
        -- print(m[9],"  ",m[10],"  ",m[11],"  ",m[12],"  ")
        
        vy=sim.multiplyVector(m,vy)
        alphaE=(vy[3]-m[12])
        
        --alphaCorr=0.25*alphaE+2.1*(alphaE-pAlphaE)
        alphaCorr=0.25*alphaE +2.1*(alphaE-pAlphaE) 
        betaE=(vx[3]-m[12])

        --betaCorr=-0.25*betaE-2.1*(betaE-pBetaE)
        betaCorr=-0.25*betaE -2.1*(betaE-pBetaE)
        pAlphaE=alphaE
        pBetaE=betaE
        cumulA=cumulA+sp[2]
        cumulB=cumulB+sp[1]
        threshold=0.5
        if (math.abs(psp2)>threshold and math.abs(sp[2])<threshold) then
            cumulA=0
        end
        if (math.abs(psp1)>threshold and math.abs(sp[1])<threshold) then
            cumulB=0
        end
        
        kp=0.08
        ki=0.005
        kd=3
        --alphaCorr=alphaCorr+sp[2]*0.005+1*(sp[2]-psp2)
        --betaCorr=betaCorr-sp[1]*0.005-1*(sp[1]-psp1)
        alphaCorr = alphaCorr + kp*sp[2] + kd*(sp[2]-psp2) + ki*cumulA
        betaCorr  = betaCorr  - kp*sp[1] - kd*(sp[1]-psp1) - ki*cumulB
        
        psp2=sp[2]
        psp1=sp[1]

        
        -- Rotational control:
        euler=sim.getObjectOrientation(d,targetObj)
        des_euler=sim.getObjectOrientation(targetObj,-1)

        rotCorr=euler[3]*0.1+2*(euler[3]-prevEuler)
        prevEuler=euler[3]
        
        
        vel[0] = thrust*(1-alphaCorr+betaCorr+rotCorr)
        vel[1] = thrust*(1-alphaCorr-betaCorr-rotCorr)
        vel[2] = thrust*(1+alphaCorr-betaCorr+rotCorr)
        vel[3] = thrust*(1+alphaCorr+betaCorr-rotCorr)
    else
        if (first_ROS_ctrl == false) then
            print("ROS control Activated")
            first_ROS_ctrl = true
        end
        
    end
    
    -- DEBUGGING
    -- print("thrust",thrust," alphaCorr",alphaCorr," betaCorr",betaCorr," rotCorr",rotCorr)
    -- print("pos:",pos )
    -- print("l:",l)
    -- print("Euler:",euler)
    -- print("vy[3]: ",vy[3],"  m[12]: ",m[12])
    -- print("alphaE:",alphaE,"betaE:",betaE)
    -- print(vel[0],"   ",vel[1],"   ",vel[2],"   ",vel[3])
    -- print("ROTOR RPMs:",vel[0]," ",vel[1]," ",vel[2]," ",vel[3])
    -- print("sp",sp)
    
    -- Decide of the motor velocities:
    handlePropeller(1 , vel[0] )
    handlePropeller(2 , vel[1] )
    handlePropeller(3 , vel[2] )
    handlePropeller(4 , vel[3] )
end 

function rotors_ctrl_cb(msg)
    -- rotors control subscriber callback
    CONTROL_FROM_ROS=true 
    
    
    vel[0] = msg.data[1]
    vel[1] = msg.data[2]
    vel[2] = msg.data[3]
    vel[3] = msg.data[4] 
end

function desired_pos_ctrl_cb(msg)
    -- Right motor speed subscriber callback
    sim.setJointTargetVelocity(rightMotor,msg.data)
end


function handlePropeller(index,particleVelocity)
    propellerRespondable=propellerHandles[index]
    propellerJoint=jointHandles[index]
    propeller=sim.getObjectParent(propellerRespondable)
    particleObject=particleObjects[index]
    maxParticleDeviation=math.tan(particleScatteringAngle*0.5*math.pi/180)*particleVelocity
    notFullParticles=0

    local t=sim.getSimulationTime()
    sim.setJointPosition(propellerJoint,t*10)
    ts=sim.getSimulationTimeStep()
    
    m=sim.getObjectMatrix(propeller,-1)
    particleCnt=0
    pos={0,0,0}
    dir={0,0,1}
    
    requiredParticleCnt=particleCountPerSecond*ts+notFullParticles
    notFullParticles=requiredParticleCnt % 1
    requiredParticleCnt=math.floor(requiredParticleCnt)
    while (particleCnt<requiredParticleCnt) do
        -- we want a uniform distribution:
        x=(math.random()-0.5)*2
        y=(math.random()-0.5)*2
        if (x*x+y*y<=1) then
            if (simulateParticles) then
                pos[1]=x*0.08
                pos[2]=y*0.08
                pos[3]=-particleSize*0.6
                dir[1]=pos[1]+(math.random()-0.5)*maxParticleDeviation*2
                dir[2]=pos[2]+(math.random()-0.5)*maxParticleDeviation*2
                dir[3]=pos[3]-particleVelocity*(1+0.2*(math.random()-0.5))
                pos=sim.multiplyVector(m,pos)
                dir=sim.multiplyVector(m,dir)
                itemData={pos[1],pos[2],pos[3],dir[1],dir[2],dir[3]}
                sim.addParticleObjectItem(particleObject,itemData)
            end
            particleCnt=particleCnt+1
        end
    end
    -- Apply a reactive force onto the body:
    totalExertedForce=particleCnt*particleDensity*particleVelocity*math.pi*particleSize*particleSize*particleSize/(6*ts)
    force={0,0,totalExertedForce}
    m[4]=0
    m[8]=0
    m[12]=0
    force=sim.multiplyVector(m,force)
    local rotDir=1-math.mod(index,2)*2
    torque={0,0,rotDir*0.002*particleVelocity}
    torque=sim.multiplyVector(m,torque)
    sim.addForceAndTorque(propellerRespondable,force,torque)
end