void moveForward(double dist, int spd) {// (cm, pct)
  struct st {
    double lt, lb;// neg = forward
    double rt, rb;// pos = forward
  };
  st curWheels = {LT.position(turns), LB.position(turns), RT.position(turns), RB.position(turns)};
  double rotNeed = dist/(10.16*3.1416);
  LT.setVelocity(-spd, pct);
  LB.setVelocity(-spd, pct);
  RT.setVelocity(spd, pct);
  RB.setVelocity(spd, pct);
  while (RT.position(turns) < curWheels.rt+rotNeed || RB.position(turns) < curWheels.rb+rotNeed || LT.position(turns) > curWheels.lt-rotNeed || LB.position(turns) > curWheels.lb-rotNeed) {
    wait(5, msec);
  }
}