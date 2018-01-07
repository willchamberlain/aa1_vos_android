%% u,v  ,  x,y,z 

observations=[
63.65524554800667, 240.81134284782155  ,  0.6042801703487033, -0.38557413463855894, 0.18;
93.00585554651721, 156.599109117345  ,  1.314108955278482, -0.2518507844260762, 0.18;
32.21257043221903, 130.32911368261986  ,  1.602826004354819, 0.3453200520455068, 0.17999999999999997;
130.2900222786177, 72.43010988987041  ,  3.0469820699928802, 0.24538560023092526, 0.18000000000000002;
220.53971640586747, 82.15134665239762  ,  2.7813816540724963, -0.648611505196154, 0.17999999999999997;
290.7330010466028, 233.72757842325063  ,  1.0825608373686624, -0.8612878684739209, 0.18000000000000002;
316.0722728878227, 100.17721606951928  ,  2.43373554369802, -1.2774049910000476, 0.18;
309.79379098189685, 149.2138430523311  ,  1.7483179195226, -1.0638150479102144, 0.18;
211.48756668529987, 57.0721476383677  ,  2.9810330787495953, -0.5511793526995465, 0.35999999999999993;
293.5929709731595, 107.065810192719  ,  1.90351526984528, -0.9655196449154599, 0.36;
270.7363579408803, 168.01371504320207  ,  1.2032554664247643, -0.8517941965515944, 0.36;
296.65479461290374, 67.95989352990506  ,  2.679197260834466, -1.143264821035397, 0.36000000000000015;
51.555458309281605, 100.49549797220861  ,  1.716291543671955, 0.2191234935049845, 0.35999999999999993;
100.61993801456948, 190.88633596565413  ,  0.6796433297836942, -0.5376280533647008, 0.36;
249.89018532564288, 214.09108135359736  ,  0.9488938806907607, -0.7323779502837205, 0.36;
53.2348468098122, 58.24917133583006  ,  1.867141273101607, 0.2311384479785062, 0.6149999999999999;
107.11574013066162, 117.0138315635702  ,  0.8281699002419981, -0.5666083547415758, 0.615;
133.21831021087385, 25.689718603738733  ,  3.2430633810027807, 0.23799345225529775, 0.615;
316.5084726745486, 35.30060533393304  ,  2.6324937354926656, -1.2872050194535172, 0.6150000000000002;
321.7155179140841, 62.91105678739576  ,  1.9070019809734335, -1.1168069307180308, 0.615;
310.91335497305147, 109.54563999105363  ,  1.2111560772533456, -1.0029152749904915, 0.615;
295.9953940592898, 145.42442997112434  ,  0.9798382519034857, -0.8805077781046192, 0.615;
];

pixel_coordinates = observations( 1: 8, 1:2);
pixel_coordinates = observations( 9:15, 1:2);
pixel_coordinates = observations(16:22, 1:2);

pixel_coordinates = observations(:,1:2);


%%
pixel_coordinates(:,2) = 190-pixel_coordinates(:,2);

%%
handle_f_2d = figure;
axis equal;
hold on
plot(pixel_coordinates(:,1),pixel_coordinates(:,2),'b');
plot(pixel_coordinates(:,1),pixel_coordinates(:,2),'bx');
axis ([0 352 0 240])
%  axis ([100 325 0 150])
axis equal

for i_ = 1:size(pixel_coordinates,1) 
    text(pixel_coordinates(i_,1),pixel_coordinates(i_,2),strcat('\color{magenta} ',num2str(i_)))
end

world_coordinates = observations( 1: 8, 3:5);
world_coordinates = observations( 9:15, 3:5);
world_coordinates = observations(16:22, 3:5);

world_coordinates = observations(:,3:5);

handle_f_3d = figure;
axis equal;
plot3( world_coordinates(:,1) , world_coordinates(:,2) , world_coordinates(:,3) , 'bx');
hold on;
plot3( world_coordinates(:,1) , world_coordinates(:,2) , world_coordinates(:,3) );
axis equal;
%axis ([0 2.2 0 2.7 0 1.0])
axis equal;
xlabel('X');  ylabel('Y');  zlabel('Z');  grid on;



cameraParams_Intrinsics = [
    519.90320*(352/640)         0    320*(352/640);
         0    518.95*(288/240)    120*(288/240);
         0         0    1 ];
     
[Re,Te,Xce,best_solutione]=efficient_pnp( world_coordinates, pixel_coordinates, cameraParams_Intrinsics );  
R = Re; T = Te; Xc = Xce; best_solution = best_solutione;
draw_axes(R, T, '\color{magenta} eff pnp', 1);

     
     
     
     