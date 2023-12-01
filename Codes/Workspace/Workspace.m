
L(1) = Link ( [ 0, 131.9, 0, pi/2, 0 ] );
L(2) = Link ( [ 0, 0, 217, 0, 0 ] );
L(3) = Link ( [ 0, 0, 185, 0, 0 ] );
L(4) = Link ( [ 0, 0, 145.6, 0, 0 ] );
robot = SerialLink(L);
robot.name = 'RIVO 4DOF Robot';
for q1 = -pi/2:0.5:pi/2
 for q2 = 0:0.5:pi
    for q3 = -pi:0.5:0
      for q4 = -pi/2:0.5:pi/2
        robot.plot ([q1,q2,q3,q4], 'workspace', [-700 700 -700 700 0 700])
        hold on
        [T,A] = robot.fkine([q1,q2,q3,q4]);
        trplot(A(4), 'frame', num2str(4))
        drawnow;
      end 
    end
 end
end