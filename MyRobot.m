classdef MyRobot < handle
    %MyRobot Create a simulated differential drive robot
    %   ROBOT = ExampleHelperDifferentialDriveRobot returns a robot object to be
    %   used in the examples. This class creates a plot for simulated robot position
    %   and computes the next position of the robot using the equations of motions
    %   of a differential drive robot. The robot has states: [x y theta], which
    %   defines the Pose of the robot
    
    % Copyright 2014 The MathWorks, Inc.
    
    properties(SetAccess = public)
        %Dt Time step for simulation of the robot
        Dt = 0.1;
        robot_color = 'r';
    end
    
    properties(SetAccess = private)
        %CurrentPose Current pose of the robot
        CurrentPose;
        
        %Trajectory Pose history of the robot
        Trajectory = [];
    end
    
    properties(Access = public)
        %HeadingVectorLength Length of the heading vector for plotting
        HeadingVectorLength = 0.5;
        
        %HRobot Graphics handle for robot
        HRobot;
        
        %HRobotHeading Graphics handle for robot heading
        HRobotHeading;
        
        %HRobotTrail Graphics handle for robot pose history
        HRobotTrail;
    end
    
    methods
        function obj = MyRobot(currentPose, robot_color, size)
            %MyRobot Constructor
            
            obj.robot_color = robot_color;
            obj.HeadingVectorLength = size;
            
            obj.CurrentPose = currentPose;
            robotCurrentLocation = obj.CurrentPose(1:2);
            
            obj.Trajectory = obj.CurrentPose;
            
            % Get the current axes
            ax = gca;
            hold(ax, 'on');
            
            % Get robot x, y data for plotting
            [x, y] = obj.getCircleCoordinate(robotCurrentLocation, obj.HeadingVectorLength);
            
            % Draw robot
            obj.HRobot = fill(x,y, obj.robot_color, 'Parent', ax);
            
            % Draw heading vector
            [xVec, yVec] = obj.computeHeadingVector(obj.CurrentPose, obj.HeadingVectorLength);
            obj.HRobotHeading = plot(ax, xVec, yVec, 'k', 'LineWidth', 3);
            
            % Draw robot trail
            obj.HRobotTrail = plot(ax, robotCurrentLocation(1),  robotCurrentLocation(2), 'LineWidth', 1, 'color', obj.robot_color);
            
             hold(ax, 'off');
        end
        
        function drive(obj, v, w)
            %drive Drive the robot by integrating the kinematics
            %   drive(OBJ, V, W) updates the current pose of the robot by
            %   integrating the equations of the motion with linear
            %   velocity input V and angular velocity input W.
            curPose = obj.CurrentPose;
            x = curPose(1) + v*cos(curPose(3))*obj.Dt;
            y = curPose(2) + v*sin(curPose(3))*obj.Dt;
            theta = curPose(3) + w*obj.Dt;
            obj.CurrentPose = [x, y, theta];
            obj.Trajectory = [obj.Trajectory; obj.CurrentPose];
            updatePlots(obj);
        end
        
        function clear_traj(obj)
            delete(obj.HRobotTrail);
            delete(obj.HRobot);
            delete(obj.HRobotHeading);
%             obj.Trajectory = [];
            ax = gca;
            hold(ax, 'on');
            robotCurrentLocation = obj.CurrentPose(1:2);
            % Get robot x, y data for plotting
            [x, y] = obj.getCircleCoordinate(robotCurrentLocation, obj.HeadingVectorLength);
            
            % Draw robot
            obj.HRobot = fill(x,y, obj.robot_color, 'Parent', ax);
            
            % Draw heading vector
            [xVec, yVec] = obj.computeHeadingVector(obj.CurrentPose, obj.HeadingVectorLength);
            obj.HRobotHeading = plot(ax, xVec, yVec, 'k', 'LineWidth', 3);
            
            % Draw robot trail
            obj.HRobotTrail = plot(ax, robotCurrentLocation(1),  robotCurrentLocation(2), 'LineWidth', 1, 'color', obj.robot_color);
            
             hold(ax, 'off');
        end
        
    end
    
    methods(Access = private)
        function updatePlots(obj)
            %updatePlots Update all plots
            
            updateHeading(obj);
            updateRobotCurrentLocation(obj);
            updateRobotTrail(obj);
            drawnow;
        end
        function updateHeading(obj)
            %updateRobotCurrentLocation Update the X/YData for heading vector
            [xVec, yVec] = obj.computeHeadingVector(obj.CurrentPose, obj.HeadingVectorLength);
            obj.HRobotHeading.XData = xVec;
            obj.HRobotHeading.YData = yVec;
        end
        function updateRobotCurrentLocation(obj)
            %updateRobotCurrentLocation Update the X/YData for robot plot
            [robotXData, robotYData] = obj.getCircleCoordinate(obj.CurrentPose, obj.HeadingVectorLength);
            obj.HRobot.XData =  robotXData;
            obj.HRobot.YData =  robotYData;
        end
        function updateRobotTrail(obj)
            %updateRobotTrail Update the X/YData for pose history
            robotCurrentLocation = obj.CurrentPose(1:2);
            obj.HRobotTrail.XData = [obj.HRobotTrail.XData  robotCurrentLocation(1)];
            obj.HRobotTrail.YData = [obj.HRobotTrail.YData  robotCurrentLocation(2)];
        end
    end
    
    methods(Access = private, Static)
        function [xVec, yVec] = computeHeadingVector(robotCurrentPose, robotDiameter)
            %computeHeadingVector Compute XData and YData for heading vector
            
            robotCurrentLocation = robotCurrentPose(1:2);
            cth = cos(robotCurrentPose(3));
            sth = sin(robotCurrentPose(3));
            
            dx = robotDiameter * cth;
            dy = robotDiameter * sth;
            
            xVec = [ robotCurrentLocation(1)   robotCurrentLocation(1)+dx];
            yVec = [ robotCurrentLocation(2)   robotCurrentLocation(2)+dy];
        end
        
        function [x, y] = getCircleCoordinate(pose, r)
            %getCircleCoordinate Compute XData and YData for a circle
            %   This function computes the XData and YData for updating the
            %   graphics object.
            theta = linspace(0,2*pi,20);
            x = r*cos(theta) + pose(1);
            y = r*sin(theta) + pose(2);
        end
        
    end
    
end

