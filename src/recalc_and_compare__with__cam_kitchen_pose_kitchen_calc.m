

scaled_360_pixels_ = [
    230.522057060657,177.970510668904;
    168.821026132822,209.516902421482;
    300.573603452410,277.712778710141;
    354.388036442100,230.393191081276;
    378.047830256533,224.826180771997;
    381.295252936946,180.754015823544;
    385.470510668904,134.362263246224;
    258.357108607049,100.960201390554;
    261.604531287461,141.784943658595;
    263.924118916327,180.290098297770;
    390.109685926636,87.9705106689044;
    255.109685926636,60.5993766482858];

pixel_coordinates = horzcat( ...
                        scaled_360_pixels_(:, 1).*(352.0/640.0) , ...
                        288-scaled_360_pixels_(:, 2).*(288.0/360.0) , ...
                        ones(size(scaled_360_pixels_,1),1) )

world_points_ = [
0,0,0;
470,0,0;
470,1000,0;
0,1000,0;
50,1170,140;
50,1170,400;
50,1170,640;
50,330,140;
50,330,400;
50,330,640;
50,1170,860;
50,330,860]

world_coordinates = horzcat ( world_points_ , ones(size(world_points_,1),1)   )

world_coordinates(:,1) = (6*320) - world_coordinates(:,1);  % ???
world_coordinates(:,2) = (6*320) - world_coordinates(:,2)  ;  % ???


%%
handle_f_2d = figure;
axis equal;
hold on
%plot(pixel_coordinates(:,1),pixel_coordinates(:,2),'b');
plot(pixel_coordinates(:,1),pixel_coordinates(:,2),'bx');
axis ([0 352 0 288])
%  axis ([100 325 0 150])
axis equal

for i_ = 1:size(pixel_coordinates,1) 
    text(pixel_coordinates(i_,1),pixel_coordinates(i_,2),strcat('\color{magenta} ',num2str(i_)))
end



handle_f_3d = figure;
axis equal;
plot3( world_coordinates(:,1) , world_coordinates(:,2) , world_coordinates(:,3) , 'bx');
hold on;
plot3( world_coordinates(:,1) , world_coordinates(:,2) , world_coordinates(:,3) );
axis equal;
%axis ([0 2.2 0 2.7 0 1.0])
axis equal;
xlabel('X');  ylabel('Y');  zlabel('Z');  grid on;

%%

cameraParams_Intrinsics = [
    519.90320         0    320;
         0       518.95    240;
         0            0      1 ];
     

cameraParams_IntrinsicMatrix_t_scaled_a = cameraParams_Intrinsics;
cameraParams_IntrinsicMatrix_t_scaled   = cameraParams_Intrinsics;

cameraParams_IntrinsicMatrix_t_scaled(1,1) = cameraParams_IntrinsicMatrix_t_scaled_a(1,1)*(352/640);
cameraParams_IntrinsicMatrix_t_scaled(1,3) = cameraParams_IntrinsicMatrix_t_scaled_a(1,3)*(352/640);
cameraParams_IntrinsicMatrix_t_scaled(2,2:3) = cameraParams_IntrinsicMatrix_t_scaled_a(2,2:3).*(288/360);     
     
     
[Re,Te,Xce,best_solutione]=efficient_pnp( world_coordinates, pixel_coordinates, cameraParams_Intrinsics );  
R = Re; T = Te; Xc = Xce; best_solution = best_solutione;
draw_axes(R, T, '\color{magenta} eff pnp', 500);