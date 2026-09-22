classdef WLRobot < handle
    %WLRobot Summary of this class goes here
    %Detailed explanation goes here

    properties
        name
        numberOfLegs
        %bodySize
        tree
        floatingBaseName

        basePosition    %not working now. maybe in 2024a
        baseOrientation
        legSet
        activeLegs
        contactLegs
        rWheel

        %limits
        Tmax %max torque for leg joints

%         currentConfiguration
        state

        ikSolver
        legSolvers
    end

    methods
        function obj = WLRobot(name)
            %WLRobot Construct an instance of this class
            %   Detailed explanation goes here
            obj.name = name;
            obj.tree = robotics.RigidBodyTree('DataFormat','row');
%             obj.tree.Gravity = [0 0 -1.625];
            obj.tree.Gravity = [0 0 -9.8];
            obj.floatingBaseName = "floatingBase";
            load("roverMassData.mat");
            floatingBaseBody = rigidBody(obj.floatingBaseName);
             J_i = J{1};
             C_i = C{1};
             M_i = M{1};            
             floatingBaseBody.Mass = M_i(1);
             floatingBaseBody.CenterOfMass = [C_i(1) C_i(2) C_i(3)];
             floatingBaseBody.Inertia = [J_i(1,1) J_i(2,2) J_i(3,3) J_i(1,2) J_i(2,3) J_i(1,3)];            
                
            floatingBaseBody.Joint = rigidBodyJoint("floatingJoint",'fixed'); %to be made floating in 2024a?
            addBody(obj.tree,floatingBaseBody,obj.tree.BaseName);
            obj.basePosition = [0 0 0];
            obj.baseOrientation = eul2quat([0 0 0]);
            obj.legSet = {'FR','RR','RL','FL'};
            obj.activeLegs = [0 0 0 0];
            obj.contactLegs = [1 1 1 1];
            obj.state = State(4,4); %each row is a leg
            obj.numberOfLegs = 0;
            obj.Tmax = [200 100 50 50];
        end

        function CreateKinematicTree(obj,A,D,Alpha,Theta,Names)
            %CreateKinematicTree creates kinematic tree
            %   inputs: DH parameters and names of the legs
            Lmax = length(A);
            solverParameters.MaxIterations = 5;
            rW = [-1 1 1 -1]*obj.rWheel;
            for L = 1 : Lmax
                a = A{L};
                d = D{L};
                alpha = Alpha{L};
                theta = Theta{L};
                legName = Names{L};
                leg = obj.createLeg(a,d,alpha,theta,legName,rW(L));
                addSubtree(obj.tree,obj.floatingBaseName,leg);
                
                obj.legSolvers{L} = inverseKinematics('RigidBodyTree',leg,'SolverAlgorithm','BFGSGradientProjection','SolverParameters',solverParameters);
            end       
            obj.ikSolver = inverseKinematics('RigidBodyTree',obj.tree);         
        end

        function SetFloatingPose(obj,position,orientation)
            obj.basePosition = position;
            obj.baseOrientation = orientation;
        end

        function SetJointLimits(obj,leg,joint,limits)
%             XXX
        end

        function SetCurrentConfiguration(obj,config)
            obj.state.position = config;
        end

        function eV = FindWheelbase(obj)
            r = [];
            N = length(obj.legSet);
            Q = obj.state.position;

            for lInd = 1 : N
                legName = obj.legSet{lInd};
                treeBaseName = strcat('b',legName,'1');
                leg = subtree(obj.tree,treeBaseName); 
                leg.Gravity = obj.tree.Gravity;
            
                q = Q(lInd,:);
                
                pointName = strcat('b',legName,'3');
                T = getTransform(leg,q,pointName);
                rInd = T(1:3,4)'; 
                r = [r; rInd];
            end
            n = obj.FindPlane(r); %n is the plane normal / abc
            
            t = [1; 0; -n(1)/n(3)];
            t = t/norm(t);

            eV = zeros(N,1);
            for lInd = 1 : N
                eNext = dot(r(lInd,:),t);
                eV(lInd) = eNext;
            end
        end

        function [abc, d, z] = FindPlane(obj,pts)
            %pts = randn(100,3) * [1 2 3;1 1 1;-1 0 1] + randn(100,3)/5 + 5;
            mu_pts = mean(pts,1);
            pts_sub = bsxfun(@minus,pts,mu_pts);
            [U,S,V] = svd(pts_sub,0);
            abc = V(:,3);
            if (dot(abc,[0;0;1])) < 0
                abc = -abc;
            end
            d = -dot(mu_pts,abc);
        
            z = mean(pts(:,3));
        end

        function totalState = GetTotalState(obj)
            wheelbase = obj.FindWheelbase();
            totalState = [obj.state.bodyPosition; obj.state.bodyOrientation; wheelbase];
            totalState(3) = obj.FindHeight();
        end        

        function z = FindHeight(obj)
            %XXX по-честному, всё не так
            r = [];
            N = length(obj.legSet);
            Q = obj.state.position;

            Tbase = [eul2rotm(flip(obj.state.bodyOrientation')) obj.state.bodyPosition;
                        0 0 0 1];

            for lInd = 1 : N
                treeBaseName = strcat('b',obj.legSet{lInd},'1');
                leg = subtree(obj.tree,treeBaseName); 
                leg.Gravity = obj.tree.Gravity;
            
                q = Q(lInd,:);
                
                pointName = strcat('b',obj.legSet{lInd},'3');
                T = getTransform(leg,q,pointName);
                rInd = T(:,4); 
                rInd = Tbase*rInd;
                r = [r; rInd(1:3)'];     
            end

            n = obj.FindPlane(r); %n is the plane normal / abc
            pts = obj.FindContactPoints(r,n,obj.rWheel);
                   
            zFloor = mean(pts(:,3));
            z = obj.state.bodyPosition(3) - zFloor;
        end        

        function [L,J] = FindRoverMatrices(obj)
            r = [];
            N = length(obj.legSet);
            Q = obj.state.position;
            angles = obj.state.bodyOrientation';
            %XXX
            angles = flip(angles);

            for lInd = 1 : N
                treeBaseName = strcat('b',obj.legSet{lInd},'1');
                leg = subtree(obj.tree,treeBaseName); 
                leg.Gravity = obj.tree.Gravity;
            
                q = Q(lInd,:);
                
                pointName = strcat('b',obj.legSet{lInd},'3');
                T = getTransform(leg,q,pointName);
                rInd = T(1:3,4)'; 
                r = [r; rInd];
            
        %         joints = [1 2 4];
        %         J_teta = [];
        %         for jInd = 1 : length(joints)
        %             jName = strcat('b',WLrover.legSet{lInd},num2str(joints(jInd)));
        %             Tj = getTransform(leg,q,jName);
        %             zJ = Tj*[0 0 1 0]';
        %             zJ = zJ(1:3);
        %             aJ = Tj(1:3,4);
        %             J_next = cross(zJ,aJ);
        %             
        %             J_teta = [J_teta J_next];
        %         end
                wheelName = strcat('Wh',obj.legSet{lInd});
                q(end) = 0;
                J_teta = geometricJacobian(leg,q,wheelName);
                J_teta = J_teta(4:6,[1 2 4]);
                Jleg{lInd} = J_teta;
        
                Ji = geometricJacobian(leg,q,pointName);
                Je{lInd} = -[Ji(4,1:2) 0]; %XXX minus?        
            end
            
            [n, d] = obj.FindPlane(r); %n is the plane normal / abc
            contactPts = obj.FindContactPoints(r,n,obj.rWheel);
        
            baseYaw = angles(1);
            basePitch = angles(2);
            baseRoll = angles(3);

            baseR = eul2rotm(angles); %yaw pitch roll
            cT = cos(baseYaw); sT = sin(baseYaw);
            cPsi = cos(basePitch); sPsi = sin(basePitch);
            cPhi = cos(baseRoll); sPhi = sin(baseRoll);
            baseTphi = [cT*cPsi -sT*cPhi+cT*sPsi*sPhi sT*sPhi+cT*sPsi*cPhi;
                        sT*cPsi cT*cPhi+sT*sPsi*sPhi -cT*sPhi+sT*sPsi*cPhi;
                        -sPsi   cPsi*sPhi            cPsi*cPhi];
            
            for lInd = 1 : N
                Sp_i = obj.findSkew(contactPts(lInd,:));
                Li{lInd} = [baseR' -Sp_i*baseTphi];
            
                t = [1; 0; 0]; %XXX - interpolate wheel center trajectory
                l = cross(n,t);
                Ri{lInd} = [t l n];
            end
            
            LTotal = [];
            JTotal = [];
            for lInd = 1 : N
                L_i = Ri{lInd}'*Li{lInd};
                LTotal = [LTotal; L_i];

                Ji = Jleg{lInd};
                s1  = size(JTotal);
                s2 = size(Ji);
            
                JTotal = [JTotal zeros(s1(1),s2(2));
                            zeros(s2(1),s1(2)) Jleg{lInd}];
            end
            
            sx = [1 0 0; 0 0 1];
            Sx = blkdiag(sx,sx,sx,sx);
            
            % su = [eye(2) zeros(2);
            %     zeros(1,3) 1];
            % Su = blkdiag(su,su,su,su);
            
            JJe = blkdiag(Je{1},Je{2},Je{3},Je{4});
            
            LL_ = Sx*LTotal;
            sL = size(LL_);
            L = [LL_ zeros(sL(1),N);
                    zeros(N,sL(2)) eye(N)];
            J = [Sx*JTotal; JJe];
        end    

        function X = findSkew(obj,x)
            X=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];
        end

        function ExecuteMotion(obj,motion,direction)
            switch motion
                case 'Hip'
                    col = 1;
                    type = 'joint';
                case 'Knee'
                    col = 2;
                    type = 'joint';
                case 'Steer'
                    col = 3;
                    type = 'joint';
                case 'Wheel'
                    col = 4;
                    type = 'joint';
                case 'Along'
                    v = [1 0 0]; %x z phi
                    type = 'cartesian';
                case 'Up-Down'
                    v = [0 1 1];
                    type = 'cartesian';                    
                case 'Bend' %not possible without body motion
                    v = [0 0 1];
                    type = 'cartesian';
            end            

            switch type
                case 'joint'
                    stepSize = 0.05; %XXX
                    qJ = obj.state.position(:,col);
                    dq = obj.activeLegs*stepSize*direction;
                    qJ = qJ+dq';
                    obj.state.position(:,col) = qJ(:);
                case 'cartesianOld'
                    %Q = obj.currentConfiguration;
                    Q = reshape(obj.state.position',[1,16]);
                    stepSize = 0.01;
                    for L = 1 : length(obj.activeLegs)
                        if (obj.activeLegs(L))
                            wheelName = strcat('b',obj.legSet{L},'3');
                            weights = ones(1,6);
                            Tprev = getTransform(obj.tree, Q, wheelName);
                            T = Tprev;
                            T(1,4) = T(1,4)+v(1)*stepSize*direction;
                            T(3,4) = T(3,4)+v(2)*stepSize*direction;
                            [qDes,solnInfo] = obj.ikSolver(wheelName,T,weights,Q);
                            indHip = (L-1)*4+1;
                            obj.state.position(L,1) = qDes(indHip);
                            obj.state.position(L,2) = qDes(indHip+1);
                        end
                    end
                case 'cartesian'
                    q = obj.state.position;
                    stepSize = 0.01;
                    for L = 1 : length(obj.activeLegs)
                        if (obj.activeLegs(L))
                            solver = obj.legSolvers{L};
                            controlBodyName = strcat('b',obj.legSet{L},'3');
                            treeBaseName = strcat('b',obj.legSet{L},'1');
                            weights = [0 0 0 1 0 1];
                            %a = tic;
                            legTree = subtree(obj.tree,treeBaseName);
                            %a1 = toc(a)
                            Tprev = getTransform(legTree, q(L,:), controlBodyName);
                            T = Tprev;
                            T(1,4) = T(1,4)+v(1)*stepSize*direction;
                            T(3,4) = T(3,4)+v(2)*stepSize*direction;
                            [qDes,solnInfo] = solver(controlBodyName,T,weights,q(L,:));
                            %a2 = toc(a)
                            obj.state.position(L,1) = qDes(1);
                            obj.state.position(L,2) = qDes(2);
                        end
                    end
            end
        end

        function contactPts = FindContactPoints(obj,r,n,Rw)
            contactPts = [];
            for lInd = 1 : size(r,1)
                cPoint = r(lInd,:)'-Rw*n;
                contactPts = [contactPts; cPoint'];
            end
        end     

        function wWheelDes = FindWheelDesiredSpeed(obj,vDes,O) 
            
            wheelCenters = obj.state.contactPts; %ignoring Z
            roverPosition = obj.state.bodyPosition; 
            x = roverPosition(1); y = roverPosition(2);
          
            for ind = 1 : size(wheelCenters,1)
                whC = wheelCenters(ind,1:2)';
                radius = norm(whC-O);
                rTurn(ind) = radius;
            end
        
            rC = norm([x; y]-O);
            wC = vDes/rC;
            wWheelDes = wC*rTurn/obj.rWheel;            
        end

        function tauRequired = SolveIDController(obj,qDes,qdDes,Kp,Kd,withForce)
            L = size(qDes,1);
            %J = size(qDes,2);
            actualPosition = obj.state.position;
            actualSpeed = obj.state.speed; 

            for lInd = 1 : L
                treeBaseName = strcat('b',obj.legSet{lInd},'1');
                leg = subtree(obj.tree,treeBaseName); 
                leg.Gravity = obj.tree.Gravity;
    
                q = actualPosition(lInd,:);
                qd = actualSpeed(lInd,:);
                desiredAcceleration = Kp(L,:).*(qDes(lInd,:)-q)+Kd(L,:).*(qdDes(lInd,:)-qd);
    
                if isnan(desiredAcceleration)
                    disp('desireAcceleration = NaN')
                end
                if withForce
                    fInd = (lInd-1)*3+1;
                    wrench = obj.state.reactionForce(fInd:fInd+2)';
                    wrench = [0 0 0 wrench];
                    wheelName = strcat('Wh',obj.legSet{lInd});
                    fExt = externalForce(leg,wheelName,wrench,q);        
                    T = inverseDynamics(leg,q,qd,desiredAcceleration,fExt);
                else 
                    T = inverseDynamics(leg,q,qd,desiredAcceleration);
                end

                T = obj.SaturateTorque(T);               
                tauRequired(lInd,:) = T(:);    
            end            
        end

        function T = SolveLegID(obj,lInd,qDes,qdDes,Kp,Kd,withForce)
            q = obj.state.position(lInd,:);
            qd = obj.state.speed(lInd,:); 

            treeBaseName = strcat('b',obj.legSet{lInd},'1');
            leg = subtree(obj.tree,treeBaseName); 
            leg.Gravity = obj.tree.Gravity;

            desiredAcceleration = Kp.*(qDes-q)+Kd.*(qdDes-qd);

            if isnan(desiredAcceleration)
                disp('desireAcceleration = NaN')
            end

            if withForce
                fInd = (lInd-1)*3+1;
                wrench = obj.state.reactionForce(fInd:fInd+2)';
                wrench = [0 0 0 wrench];
                wheelName = strcat('Wh',obj.legSet{lInd});
                fExt = externalForce(leg,wheelName,wrench,q);        
                T = inverseDynamics(leg,q,qd,desiredAcceleration,fExt);
            else 
                T = inverseDynamics(leg,q,qd,desiredAcceleration);
            end

            T = obj.SaturateTorque(T);          
        end

        function [pActual,vActual,Jleg,Jtotal] = FindLegMotion(obj,lInd)
            treeBaseName = strcat('b',obj.legSet{lInd},'1');
            leg = subtree(obj.tree,treeBaseName);            

            q = obj.state.position(lInd,:);
            qd = obj.state.speed(lInd,:); 

            pointName = strcat('b',obj.legSet{lInd},'3');
            TActual = getTransform(leg,q,pointName);
            pActual = TActual([1 3],4);  

            Jtotal = geometricJacobian(leg,q,pointName);
            Jleg = Jtotal([4 6],1:2);
            vActual = Jleg*qd(1:2)';            
        end

        function T = SolveLegIDCartesian(obj,lInd,legDyn,vDes,iVDes,Tt,gamma,s)
            [pActual,vActual,JCart,Jleg] = obj.FindLegMotion(lInd);

            xd = vActual;
            xd = [0 0 0 xd(1) 0 xd(2)]';
            xdd = zeros(size(xd));

            iT = diag(1./Tt);                                    
            Ki = legDyn.L*iT*iT;
            Ddes = 2*gamma*Ki*diag(Tt);

            xdDes = [0 0 0 vDes(1) 0 vDes(2)]';
            u6 = legDyn.L*xdd + legDyn.muX - Ddes*(xd - xdDes);

            iv = obj.state.speedIntegral(lInd,:);
            S = diag(s);
            u6 = u6 + Ki*S*(iVDes' - iv');

            T = legDyn.g + (Jleg')*u6 - legDyn.dm';
            T = obj.SaturateTorque(T'); 
            T = T';            
        end

        function T = SolveLegIDCartesianHybrid(obj,lInd,legDyn,vDes,iVDes,fDes,iFDes,Tt,gamma,s,sf,KF)
            [pActual,vActual,JCart,Jleg] = obj.FindLegMotion(lInd);

            xd = vActual;
            xd = [0 0 0 xd(1) 0 xd(2)]';
            xdd = zeros(size(xd));

            iT = diag(1./Tt);                                    
            Ki = legDyn.L*iT*iT;
            Ddes = 2*gamma*Ki*diag(Tt);

                Sf = diag(sf);
                Im = eye(size(Sf));
                Sv = Im - Sf;
                Sf = Sf(4:6,4:6);
                uDin = legDyn.L*xdd + legDyn.muX;

            xdDes = [0 0 0 vDes(1) 0 vDes(2)]';
            iv = obj.state.speedIntegral(lInd,:);
            S = diag(s);                        
            uV = -Ddes*Sv*(xd - xdDes)+ Ki*Sv*S*(iVDes' - iv');            

                fInd = (lInd-1)*3+1;
                Fa = -obj.state.reactionForce(fInd:fInd+2)';
                iFa = -obj.state.forceIntegral(fInd:fInd+2)';

                Kf = diag([1 1 1])*KF.kF;
                Kfi = diag([1 1 1])*KF.kFi;
                Kv = eye(3)*KF.kFv;
                v = xd(4:6);    
                uF = [];
                uF = fDes + Kf*Sf*(fDes-Fa') + Kfi*Sf*(iFDes'-iFa') - Kv*v;   
                uF = [0; 0; 0; uF];
        
                u6 = uDin + uV + uF;

            T = legDyn.g + (Jleg')*u6 - legDyn.dm';
            T = obj.SaturateTorque(T'); 
            %T = T';            
        end        

        function T = SaturateTorque(obj,T)
            %T is row vector!
            Tmin = -obj.Tmax;
            T = min(T,obj.Tmax);
            T = max(T,Tmin);            
        end

        function [deltaDes,rotationSign] = FindDeltaDesired(obj,O)

            localWheelCenters = obj.state.contactPts; %ignoring Z
            roverPosition = obj.state.bodyPosition;
            phi = obj.state.bodyOrientation(3);
            x = roverPosition(1); y = roverPosition(2);
            C = [x; y];
            cPhi = cos(phi);
            sPhi = sin(phi);
            Mphi = [cPhi -sPhi;
                     sPhi cPhi];            

            lwc = localWheelCenters(:,1:2)';
    wheelCenters = Mphi*lwc;
    wheelCenters(1,:) = wheelCenters(1,:) + x;   
    wheelCenters(2,:) = wheelCenters(2,:) + y;              
        
            ox = [1;0];
            aheadDirection = Mphi*ox;
            v2 = O - C;
            v2 = [v2;0];
            aheadDirection = [aheadDirection;0];
            crossP = cross(aheadDirection,v2);
            rotationSign = sign(crossP(3));
        
            N = size(wheelCenters,2);
            deltaDes = zeros(1,N);
            for ind = 1 : N
                whC = wheelCenters(:,ind);
                r = whC - O;
                phiR = atan2(r(2),r(1));
                phiN = phiR+rotationSign*pi/2;
                delta = phiN - phi;
                out = mod(delta+pi/2,pi)-pi/2;
                deltaDes(ind) = out;
                %R = norm(whC-O);
                %deltaDes(ind) = rotationSign*asin(lever(ind)/R);%+K*phiError;
            end            
        end        

        function ShowFrame(obj,config)
%             totalConfig = [obj.baseOrientation,obj.basePosition,config];
            configRow = reshape(config',[1,16]);
            show(obj.tree,configRow);
        end

        function Show(obj)
%             Q = obj.currentConfiguration
            configRow = reshape(obj.state.position',[1,16]);
            show(obj.tree,configRow);
        end        
    end
    
    methods (Access = private)
        function leg = createLeg(obj,a,d,alpha,theta,name,rWheel)
            load("roverMassData.mat");
             N = 4;
             L = obj.numberOfLegs+1;
             mass = zeros(1,4);
             CMx = zeros(1,4);
             CMy = zeros(1,4);
             CMz = zeros(1,4);
             
             Ixx = zeros(1,4);
             Iyy = zeros(1,4);
             Izz = zeros(1,4);
             Ixy = zeros(1,4);
             Iyz = zeros(1,4);
             Ixz = zeros(1,4);
             
             for jInd = 1 : 4
                    ind = (L-1)*4+jInd+1;
                     J_i = J{ind};
                     C_i = C{ind};
                     M_i = M{ind};
                     
                     mass(jInd) = M_i;
                     
                     CMx(jInd) = C_i(1);
                     CMy(jInd) = C_i(2);
                     CMz(jInd) = C_i(3);
                     
                     Ixx(jInd) = J_i(1,1);
                     Iyy(jInd) = J_i(2,2);
                     Izz(jInd) = J_i(3,3);
                     Ixy(jInd) = J_i(1,2);
                     Iyz(jInd) = J_i(2,3);
                     Ixz(jInd) = J_i(1,3);     
             end            
            leg = robotics.RigidBodyTree('DataFormat','row');
            leg.Gravity = [0 0 -1.625];
            N = length(a);
             for jInd = 1 : N
                 bodyName = strcat('b',name,num2str(jInd));
                 body = robotics.RigidBody(bodyName);
                 body.Mass = mass(jInd);
                 body.CenterOfMass = [CMx(jInd) CMy(jInd) CMz(jInd)];
                 body.Inertia = [Ixx(jInd) Iyy(jInd) Izz(jInd) Ixy(jInd) Iyz(jInd) Ixz(jInd)];
                 jointName = strcat('joint',name,num2str(jInd));
                 joint = robotics.Joint(jointName,'revolute');
                 setFixedTransform(joint,[a(jInd) alpha(jInd) d(jInd) theta(jInd)],'mdh'); %MDH?
            
                 body.Joint = joint;
                 if jInd == 1
                    baseName = leg.BaseName;
                 else
                    baseName = leg.BodyNames{jInd-1};
                 end
                 addBody(leg,body,baseName); 
             end
        
             wheelName = strcat('Wh',name);
             wheel = robotics.RigidBody(wheelName);
             pose = trvec2tform([0, rWheel, 0]); 
             setFixedTransform(wheel.Joint,pose); 
             wheel.Mass = 0.001;
             wheel.CenterOfMass = [0 0 0];
             wheel.Inertia = [0.02 0.02 0.02 0 0 0]*0.001;
             lastBodyName = leg.BodyNames{N};
             addBody(leg,wheel,lastBodyName); 
             obj.numberOfLegs = L;
        end
    end
end