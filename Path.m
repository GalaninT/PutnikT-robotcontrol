classdef Path
    %Path Summary of this class goes here
    %   Detailed explanation goes here

    properties
        x
        y 
        z
        phi
        t
        v
        alpha
    end

    methods
        function obj = Path()
            %Path Construct an instance of this class
            %   Detailed explanation goes here
            %obj.Property1 = inputArg1 + inputArg2;
        end

        function [keyPointInd,Vactual] = FindClosestPointInd(obj,state,lastKeyPointInd) 
            phiActual = state.bodyOrientation(3);
            V = state.bodySpeed;
            Vactual = dot(V(1:2),[cos(phiActual) sin(phiActual)]);                    
            s = sign(Vactual); %actual motion direction (forward/backward)

            P = state.bodyPosition(1:2);
            found = 0;
            lastInd = lastKeyPointInd;
            while ~found 
                if (lastInd+1 > length(obj.x))
                    keyPointInd = lastInd;
                    return;
                end
                PLast = [obj.x(lastInd) obj.y(lastInd)]';
                PNext = [obj.x(lastInd+1) obj.y(lastInd+1)]';
                vectorToNext = PNext - P;                
                distanceNext = norm(vectorToNext);
                vectorToLast = (PLast-P);%*sign(vectorToNext);
                distanceLast = norm(vectorToLast);
                pointDirection = dot(vectorToLast,vectorToNext);
                if (distanceLast < distanceNext)
                    if (distanceLast < distanceNext/3)
                        keyPointInd = lastInd+1;
%                     elseif abs(phiActual-pointDirection) > pi/2
                    elseif pointDirection < 0
                        keyPointInd = lastInd+1;
                    else
                        keyPointInd = lastInd;
                    end
                    return;
                else 
                    lastInd = lastInd+1;
                end
            end
        end

        function point = GetPoint(obj,pointInd)
            point.x = obj.x(pointInd);
            point.y = obj.y(pointInd);
            point.phi = obj.phi(pointInd);
            point.t = obj.t(pointInd);
            point.v = obj.v(pointInd);            
        end

%         function [R,C] = FindArcByTwoPoints(obj,keyPointInd,roverPosition,s)
%             %     %keypoint is always ahead except final part                
%             p1 = [obj.x(keyPointInd) obj.y(keyPointInd)];
%             
%             if (keyPointInd == length(obj.x))
%                 %r = norm([obj.x(keyPointInd)-obj.x(keyPointInd-1) obj.y(keyPointInd)-obj.y(keyPointInd-1)]);
%                 r = 2*s;
%                 p2(1) = obj.x(keyPointInd)+r*cos(obj.phi(keyPointInd));
%                 p2(2) = obj.y(keyPointInd)+r*sin(obj.phi(keyPointInd));
%         
%                 phiDes = obj.phi(keyPointInd);
%                 %нужно учесть направление движения, чтобы не уехать от точки???
%             else
%                 p2 = [obj.x(keyPointInd+1) obj.y(keyPointInd+1)];
%                 phiDes = obj.phi(keyPointInd+1);
%             end        
%             
%             p3 = roverPosition(1:2)';
%             
%             [R,C] = obj.FindCircleByPointAndTangent(p3,p2,phiDes,s); 
%             C = C';
%         end

        function [R,C] = FindArcByTwoPoints(obj,keyPointInd,roverPosition,s)
            if (keyPointInd == length(obj.x))
                %r = norm([obj.x(keyPointInd)-obj.x(keyPointInd-1) obj.y(keyPointInd)-obj.y(keyPointInd-1)]);
                r = 3*s;
                psiDes = obj.phi(keyPointInd) + obj.alpha;
                p2(1) = obj.x(keyPointInd)+r*cos(psiDes);
                p2(2) = obj.y(keyPointInd)+r*sin(psiDes);
            else
                psiDes = obj.phi(keyPointInd+1) + obj.alpha;
                p2 = [obj.x(keyPointInd+1) obj.y(keyPointInd+1)];
            end        
            
            p3 = roverPosition(1:2)';
            
            [R,C] = obj.FindCircleByPointAndTangent(p3,p2,psiDes,s); 
            C = C';
        end

        function [R,O] = FindCircleByPointAndTangent(obj,A,B,phi,s)
            %to path class
            xA = A(1); yA = A(2); xB = B(1); yB = B(2); 
            xM = (xA+xB)/2; yM = (yA+yB)/2;
        
            dy = s*(yB-yA); dx = s*(xB-xA);
            psi = atan2(dy,dx);
        
            if (abs(phi-psi) > 0.001)
                R = [sin(phi) -sin(psi)
                     -cos(phi) cos(psi)];
                b = [xM-xB; yM-yB];
                param = R\b;
            else
                param(1:2) = 1000*sign(psi-phi);
            end
        
            O(1) = xM + param(2)*sin(psi);
            O(2) = yM - param(2)*cos(psi);
        
            R = norm(B-O);
        end 

        function phiCorrection = FindDeltaForLine(obj,keyPointInd,roverPosition)
            %phi is rover trajectory tangent, not rover orientation            
            if (keyPointInd == length(obj.x))
                %use rover length as r XXX
                lastPointDirectionVector = [obj.x(keyPointInd)-obj.x(keyPointInd-1) obj.y(keyPointInd)-obj.y(keyPointInd-1)];
                r = norm(lastPointDirectionVector);
                alphaRequired = atan2(lastPointDirectionVector(2),lastPointDirectionVector(1));
                p2(1) = obj.x(keyPointInd)+r*cos(alphaRequired);
                p2(2) = obj.y(keyPointInd)+r*sin(alphaRequired);
            else
                p2 = [obj.x(keyPointInd+1) obj.y(keyPointInd+1)];
                prevPoint = obj.GetPoint(keyPointInd); 
                alphaRequired = atan2(p2(2)-prevPoint.y,p2(1)-prevPoint.x);
            end       
            
            p3 = roverPosition(1:2)';            
            v2 = p2-p3;

            alphaExpected = atan2(v2(2),v2(1));
            phiCorrection = alphaRequired - alphaExpected;        
        end        

        function pathProgress = FindProgress(obj,bodyPosition)
            rTotal = [obj.x(end)-obj.x(1) obj.y(end)-obj.y(1)];
            rTotalNorm = norm(rTotal);

            rActual = [bodyPosition(1)-obj.x(1) bodyPosition(2)-obj.y(1)];
            rActualNorm = norm(rActual);

            pathProgress = rActualNorm/rTotalNorm;

        end

    end
end