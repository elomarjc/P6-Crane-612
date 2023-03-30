  %No movement of L direction

  %Simulation values before adjustment
  J_l = 0.8;
  J_x = 0.13;
  g = 9.82;
  k_el = 0.71;
  k_ex = 0.609;
  m_l = 3.89;
  m_t = 0.729;
  r_l = 0.05;
  r_x = 0.075;
  B_l = 20;
  B_x = 12;
  sFrictionX=20;
  cFrictionX=18;
  sFrictionL=50;
  cFrictionL=90;
  InitWireLength=1;
  I_xStart=1;
  I_xEnd=3;
  I_xCurrent=0;

  I_LStart=1;
  I_LEnd=3;
  I_LCurrent=0.25;


  closedLoop=sim("CraneModel.slx");
  plot(closedLoop.tout, closedLoop.L)
  grid on

