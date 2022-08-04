%% general setting
%- global variables initialization
global numExplorations numExploredNodes numReexpandedNodes
numExplorations = 0; numExploredNodes = 0; numReexpandedNodes = 0;
%- deleting workspace and closing windows
clearvars; format shortG; format compact;
clc; close all

%% select the DEM type
%- accepted values: true for synthetic DEM, otherwise for real-world DEM
DEMtype = false;

%% loading synthetic or real-world DEM from MAT file
%- select the desired filename for synthetic or real-world DEM
%- accepted values: 'syntheticDEM_01', 'syntheticDEM_02', 'syntheticDEM_03', 'realworldDEM'
if DEMtype, filename = 'syntheticDEM_01'; else, filename = 'realworldDEM'; end
load(filename,'-mat');
clear filename

%% plotting the selected DEM in a 3D graph
%- setting number of colors for elevation
numberColors = 2^10;
% setting colormap of natural terrain appearance
figure(1); clf; h1 = gcf; h1.Color = 'w'; g1 = gca;
%- uncomment the next line if you have installed Mapping Toolbox for more realistic representation
% demcmap(DEM.Z,numberColors);
figure(2); clf; h2 = gcf; h2.Color = 'w'; g2 = gca;
%- uncomment the next line if you have installed Mapping Toolbox for more realistic representation
% demcmap(DEM.Z,numberColors);
%- plotting graphical representation
surf(g1,DEM.Z'); shading(g1,'interp');
surf(g2,DEM.Z'); shading(g2,'interp');
%- setting natural terrain appearance
axis(g1,[0 size(DEM.Z,1) 0 size(DEM.Z,2) min(DEM.Z(:)) max(DEM.Z(:))]);
axis(g1,'equal'); camlight(g1,'headlight');
view(g2,2); axis(g2,'equal'); axis(g2,[0 size(DEM.Z,1) 0 size(DEM.Z,2)]);
%- showing colorbar in the graphical representation
c = colorbar(g2,'westoutside');
%- showing title and axis labels in the graphical representation
title(g1,{'3D view of DEM with round trip trajectories';'Trajectory color scale for reference velocities';'(red = max, yellow = middle, and blue = min)';''});
title(g2,{'2D view of DEM with round trip trajectories';'Trajectory color scale for reference velocities';'(red = max, yellow = middle, and blue = min)';''});
xlabel(g1,'X Coordinate (meter)'); ylabel(g1,'Y Coordinate (meter)'); zlabel(g1,'Z Coordinate, Altitude (meter)');
xlabel(g2,'X Coordinate (meter)'); ylabel(g2,'Y Coordinate (meter)');
c.Label.String = 'Altitude (meter)';

%% defining characteristic parameters of the UGV
%- select the appropriate values for the desired UGV
%- correct format of accepted data: [nominalSpeed,kv_inclinationAware,COG,L,W,rho_tol] with
%-  nominaSpeed > 0
%-  kv_inclinationAware >= [0 0]
%-  COG = [x y z] where [x,y] < [L,W] and z > 0
%-  L > 0
%-  W > 0
%-  0 <= rho_tol < 1
%-
%- some accepted values:
%-  [1,[30 10],[0.00 0.03 0.60],0.68,0.62,0.29] for synthetic DEMs
%-  [1,[300 100],[0.00 0.30 0.97],2.025,1.210,0.85] for real-world DEM
%- 
if DEMtype, constraints_UGV_DEMAIA = [1,[30 10],[0.00 0.03 0.60],0.68,0.62,0.29]; else, constraints_UGV_DEMAIA = [1,[300 100],[0.00 0.30 0.97],2.025,1.210,0.85]; end
%- computing the asymmetric inclination-aware velocity constraints function, LUT
resolution = 0.001;
[LUT,theta_threshold,symmetric_threshold] = computeLUT(resolution,constraints_UGV_DEMAIA,false);
%- displaying the pitch thresholds for defined UGV
disp(['Pitch thresholds: [',num2str(theta_threshold(1)*180/pi),',',num2str(theta_threshold(2)*180/pi),'] grados']);
%- displaying the symmetric pitch and roll threshold for defined UGV (only informative data)
disp(['Symmetric threshold: ',num2str(symmetric_threshold*180/pi),' grados']);

%% plotting UGV pitch and roll stability region
figure(3); clf; h3 = gcf; h3.Color = 'w'; hold on;
negativeAngles = flip([0:-resolution:-pi/2]); negativeAngles = negativeAngles(1:end-1);
angles = 180/pi*[negativeAngles 0:resolution:pi/2];
im = imagesc(angles,angles,(LUT.table)',[0 1]);
im.AlphaData = 0.25; % grid on;
color = winter; color(1,:) = [1 1 1]; colormap(color);
title('UGV stability region');
xlabel('pitch {\bf\theta} (º)');
ylabel('roll {\bf\phi} (º)');
x = symmetric_threshold*180/pi;
plot([-x x x -x -x],[-x -x x x -x],'--r','LineWidth',0.5);
axis equal
offset = 3;
axis([-max(abs(theta_threshold)*180/pi) max(abs(theta_threshold)*180/pi) -symmetric_threshold*180/pi-offset symmetric_threshold*180/pi+offset]);
plot([-max(abs(theta_threshold)*180/pi) max(abs(theta_threshold)*180/pi)],[0 0],'-k','LineWidth',0.01);
plot([0 0],[-symmetric_threshold*180/pi-offset symmetric_threshold*180/pi+offset],'-k','LineWidth',0.01);

%% setting configuration parameters for DEM-AIA planner
%- select the appropriate values for the desired configuration of DEM-AIA planner
%- correct format of accepted data: [enabled_any_angle true heuristicFunction true] with
%-  enabled_any_angle        : 'true' for any-angle search
%-  enabled_heuristic_search : 'true' for heuristic search, otherwise H=0
%-  heuristicFuntion         : identified the heuristic function, accepted values: '1' for octile-time, otherwise Euclidean-time
%-  enabled_reexpansions     : 'true' for node re-expansion into iterative node expansion process
%-
%- some accepted values:
%-  [false,true,1,true] for DEM-AIA(n) and DEM-AIA(n,s)
%-  [true,true,1,true] for DEM-AIA and DEM-AIA(s)
%- 
features_DEMAIA = [true true 1 true];
%- select the inclination-awareness: symmetrical or asymmetrical for appropriate DEM-AIA variants
%- accepted values: 'true' for symmetrical (DEM-AIA(n,s) and DEM-AIA(s)), otherwise asymmetrical (DEM-AIA(n) and DEM-AIA)
awareness = false;
%- computing the symmetric or asymmetric inclination-aware velocity constraints function, LUT
resolution = 0.001;
[LUT,~,~] = computeLUT(resolution,constraints_UGV_DEMAIA,awareness);

%% setting the start and goal nodes for path planning
% select the start (c0) and goal (cn) nodes within the environment limits (DEM.Z matrix size)
%- correct format of accepted data: [x,y] with
%-  x <= size(DEM.Z,1)
%-  y <= size(DEM.Z,2)
%-
%- some accepted values:
%-  c0 = [10 80] and cn = [470 410] for synthetic DEMs
%-  c0 = [66 63] and cn = [203 243] or [185 98] for real-world DEM
%- 
if DEMtype, c0 = [10 80]; cn = [470 410]; else, c0 = [66 63]; cn = [203 243]; end
if DEMtype, c0 = [10 80]; cn = [470 410]; else, c0 = [66 63]; cn = [185 98]; end

%% testing configurated DEM-AIA planner for the defined UGV from desired start node (c0) to goal node (cn) over selected DEM: first trajectory
tic; waypoints_planner = DEMAIA(DEM,c0,cn,LUT,features_DEMAIA); compTime = toc;
[long,time,proposalSlopePos,slopeNeg,slopePos,tiltPos,turnAcc] = analysisWaypoints(waypoints_planner);
fprintf('--[blue line]--\n');
plotPath(constraints_UGV_DEMAIA,features_DEMAIA,c0,cn,compTime,long,time,proposalSlopePos,slopeNeg,slopePos,tiltPos,turnAcc,waypoints_planner,...
       [g1,g2],'-','b');

%% testing configurated DEM-AIA planner for the defined UGV from desired start node (cn) to goal node (c0) over selected DEM: return trajectory
tic; waypoints_planner = DEMAIA(DEM,cn,c0,LUT,features_DEMAIA); compTime = toc;
[long,time,proposalSlopePos,slopeNeg,slopePos,tiltPos,turnAcc] = analysisWaypoints(waypoints_planner);
fprintf('--[red line]--\n');
plotPath(constraints_UGV_DEMAIA,features_DEMAIA,cn,c0,compTime,long,time,proposalSlopePos,slopeNeg,slopePos,tiltPos,turnAcc,waypoints_planner,...
         [g1,g2],'-','r');





%==========================================================================================
%- LOCAL FUNCTIONS
%==========================================================================================
%------------------------------------------------------------------------------------------
%- showing information and plotting graphical representation
function plotPath( constraints_UGV,features_planner,c0,cn,compTime,long,time,proposalSlopePos,slopeNeg,slopePos,tiltPos,turnAcc,waypoints,handlesAxes,styleLine,colorLine )
 global numExplorations numExploredNodes numReexpandedNodes
 if length(constraints_UGV)==4
  fprintf(2,'UGV constraints code [%g,%g,%g,%g]\n',constraints_UGV(1:3),constraints_UGV(4)*180/pi);
 elseif length(constraints_UGV)==9
  fprintf(2,'UGV constraints code [%g,%g,%g,%g,%g,%g,%g,%g,%g]\n',constraints_UGV);
 end
 fprintf(2,'Planner features code [%g,%g,%g,%g]\n',features_planner);
 if isempty(waypoints), fprintf(2,'There are no admissible paths.\n');
 else
  fprintf('Initial and final cells: from ');
  %if isnan(c0(3))
  fprintf('(%g,%g) to ',waypoints(1,1:2));
  %else, fprintf('(%g,%g,%gº) to ',waypoints(1,1:2),c0(3)*180/pi);
  %end
  %if isnan(cn(3)),
  fprintf('(%g,%g)\n',waypoints(end,1:2));
  %else, fprintf('(%g,%g,%gº)\n',waypoints(end,1:2),cn(3)*180/pi);
  %end
  fprintf('Explored nodes: %g with %g iterations into graph search process\n',numExploredNodes,numExplorations);
  fprintf('Re-expanded nodes: %g\n',numReexpandedNodes);
  fprintf('Total path distance: %g m.\n',long);
  fprintf('Total UGV navigation time: %g s.\n',time);
  fprintf('Total average speed: %g m/s.\n',long/time);
  fprintf('Maximum proposal path slope: %+g rad. (%+g º)\n',proposalSlopePos,proposalSlopePos*180/pi);
  fprintf('Maximum path slope: %+g / %+g rad. (%+g / %+g º)\n',[slopeNeg slopePos],[slopeNeg slopePos]*180/pi);
  fprintf('Maximum tilt inclination: %+g rad. (%+g º)\n',tiltPos,tiltPos*180/pi);
  fprintf('Accumulated turn value: %g rad. (%g º)\n',turnAcc,turnAcc*180/pi);
  fprintf('Number of intersection points (waypoints over surface): %g \n',size(waypoints,1));
  %- fprintf('Detailed waypoints list: \n'); fprintf(' %6.2f \t %6.2f \t %4.4f \t %4.4f \n',path');
  fprintf('Algorithm computation time: %g s.\n',compTime);
  if ~isempty(waypoints) && ~isempty(colorLine)
   vn = constraints_UGV(1);
   color = jet(2^10);
   axes(handlesAxes(1)); hold on;
   for i=2:length(waypoints)
    colorLine = color(uint32(waypoints(i,4)*2^10),:);
    plot3([waypoints(i-1,1) waypoints(i,1)],[waypoints(i-1,2) waypoints(i,2)],[waypoints(i-1,3) waypoints(i,3)],'LineStyle',styleLine,'Color',colorLine,'Linewidth',4);
   end
   %colormap(color); c = colorbar;
   axes(handlesAxes(2)); hold on;
   for i=2:length(waypoints)
    colorLine = color(uint32(waypoints(i,4)*2^10),:);
    plot3([waypoints(i-1,1) waypoints(i,1)],[waypoints(i-1,2) waypoints(i,2)],1+[waypoints(i-1,3) waypoints(i,3)],'LineStyle',styleLine,'Color',colorLine,'Linewidth',3);
   end
   %colormap(color); colorbar;
   drawnow
  end
 end
end
