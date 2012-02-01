function doit
	doit2()





function doit2
	close all
	clear all	
	xe = 0.0:0.00001:0.3;
	ye = 0.0:0.00001:0.3;

	d0 = [pi/4, pi/2];
	d00 = d0;
	L = [0.5, 0.5];
	
	[xy_end,xy_elbow] = jacobianGetPos(d0, L);
	x0 = xy_end(1);
	y0 = xy_end(2);
	x00 = x0;
	y00 = y0;
	x = [xy_end(1), xy_elbow(1), 0];
	y = [xy_end(2), xy_elbow(2), 0];
	
	h = plot(x,y)


	axis([-1 1 0 1])
	axis square
	grid off
	set(h,'EraseMode','xor','MarkerSize',18)

while(1)
	d0 = d00;
	x00 = x0;
	y00 = y0;
	for i = 1:length(x)
		drawnow
		x1 = x0+xe(i);
		y1 = y0+ye(i);
		goFlag = 0;
		x11 = x1;
		y11 = y1;

		re0 = [0, 0];
		re1 = [0, 0];
		while( goFlag == 0)
			%% End effector Pos Now
			r0 = [ x0, y0 ];
			
			%% Goal Pos
			rg = [ x1, y1 ];

			%% Error
			re 	= rg - r0;
			re1 	= re0;
			re0 	= re;
			red 	= re0 - re1;

			%% apply gains
			kx = 1;
			ky = 1;
			kxd = 0.5;
			kyd = 0.5;

			k  = [kx, ky];
			kd = [kxd, kyd];
			rek = re.*k + re.*kd;
			
			[d1, e1] = jacobianIk2Dof( rek, L, d0 );
			ae = sum(e1.^2).^0.5;
			[xy_end,xy_elbow] = jacobianGetPos(d1, L);
			

			x0 = xy_end(1);
			y0 = xy_end(2);

	
			d0 = d1;
			disp(['Error = ', num2str(ae)])
			if(ae < 0.0000001)
				goFlag = 1;
			end
		end

		x0 = x1;
		y0 = y1;

		%x0 = xy_end(1);
		%y0 = xy_end(2);
		
		x = [xy_end(1), xy_elbow(1), 0];
		y = [xy_end(2), xy_elbow(2), 0];
		
		a = ((x(3)-x(2))^2 + (y(3)-y(2))^2)^0.5;
		b = ((x(2)-x(1))^2 + (y(2)-y(1))^2)^0.5;
		disp(['all 0.5s = ', num2str(a), '  ', num2str(b), '  x1 = ', num2str(x1),'  y1 = ', num2str(y1), '  Err = ', num2str(ae)])
		
		
		%h = plot(x,y);
		%set(h);
		axis([-1 1 0 1])
		axis square
		grid off
		set(h, 'YData',y, 'XData',x);%,'EraseMode','xor','MarkerSize',18)
		pause(0.05)
	end
		
end
	





function doit1
x=-5:.2:5;
y=sin(x);
h=plot(x,y);
axis([-2*pi 2*pi -1 1])
axis square
grid off
set(h,'EraseMode','xor','MarkerSize',18)
for t=2:.1:7
    drawnow
    y=sin(x)*exp(-t);
    set(h,'YData',y)
    pause(0.05)
end
