[1mdiff --git a/TPV_Controller.ino b/TPV_Controller.ino[m
[1mindex 71ef94e..ced5899 100644[m
[1m--- a/TPV_Controller.ino[m
[1m+++ b/TPV_Controller.ino[m
[36m@@ -7,7 +7,7 @@[m
 #include <FeedBackServo.h>[m
 [m
 #define KP -0.2[m
[31m-[m
[32m+[m[32m//Dave was here[m
 #define G_X 0.7[m
 #define G_Y 0.7[m
 [m
[36m@@ -76,77 +76,86 @@[m [mint turns2 = 0;[m
 [m
 void loop()[m
 {[m
[31m-//Calculations of the servo angle from the timings of the input signal[m
 //----------------------------------------------------------------[m
[31m-  int tCycle = tHigh1 + tLow1;[m
[31m-  if((tCycle > 1000) && (tCycle < 1200)){[m
[31m-    float dc = (dutyScale * tHigh1) / (float)tCycle;[m
[31m-    float theta = (unitsFC - 1) - ((dc - dcMin) * unitsFC) / (dcMax - dcMin + 1);[m
[31m-  [m
[32m+[m[32m  if(1){[m
[32m+[m[32m    int tCycle = tHigh1 + tLow1;[m
[32m+[m[32m    if((tCycle > 1000) && (tCycle < 1200)){[m
[32m+[m[32m      float dc = (dutyScale * tHigh1) / (float)tCycle;[m
[32m+[m[32m      float theta = (unitsFC - 1) - ((dc - dcMin) * unitsFC) / (dcMax - dcMin + 1);[m
     [m
[31m-    if(theta < 0.0)[m
[31m-        theta = 0.0;[m
[31m-    else if(theta > (unitsFC - 1.0))[m
[31m-        theta = unitsFC - 1.0;[m
[31m-  [m
[31m-    if((theta < q2min) && (thetaPre1 > q3max))[m
[31m-        turns1++;[m
[31m-    else if((thetaPre1 < q2min) && (theta > q3max))[m
[31m-        turns1--;[m
[31m-  [m
[31m-    if(turns1 >= 0)[m
[31m-        angle1 = (turns1 * unitsFC) + theta;[m
[31m-    else if(turns1 < 0)[m
[31m-        angle1 = ((turns1 + 1) * unitsFC) - (unitsFC - theta);[m
[31m-  [m
[31m-    thetaPre1 = theta;[m
[32m+[m[41m      [m
[32m+[m[32m      if(theta < 0.0)[m
[32m+[m[32m          theta = 0.0;[m
[32m+[m[32m      else if(theta > (unitsFC - 1.0))[m
[32m+[m[32m          theta = unitsFC - 1.0;[m
[32m+[m[41m    [m
[32m+[m[32m      if((theta < q2min) && (thetaPre1 > q3max))[m
[32m+[m[32m          turns1++;[m
[32m+[m[32m      else if((thetaPre1 < q2min) && (theta > q3max))[m
[32m+[m[32m          turns1--;[m
[32m+[m[41m    [m
[32m+[m[32m      if(turns1 >= 0)[m
[32m+[m[32m          angle1 = (turns1 * unitsFC) + theta;[m
[32m+[m[32m      else if(turns1 < 0)[m
[32m+[m[32m          angle1 = ((turns1 + 1) * unitsFC) - (unitsFC - theta);[m
[32m+[m[41m    [m
[32m+[m[32m      thetaPre1 = theta;[m
[32m+[m[32m    }[m
   }[m
[31m-  [m
[32m+[m
 //----------------------------------------------------------------[m
[31m-  int tCycle = tHigh2 + tLow2;[m
[31m-  if((tCycle > 1000) && (tCycle < 1200)){[m
[31m-    float dc = (dutyScale * tHigh2) / (float)tCycle;[m
[31m-    float theta = (unitsFC - 1) - ((dc - dcMin) * unitsFC) / (dcMax - dcMin + 1);[m
[31m-  [m
[32m+[m[32m  if(1){[m
[32m+[m[32m    int tCycle = tHigh2 + tLow2;[m
[32m+[m[32m    if((tCycle > 1000) && (tCycle < 1200)){[m
[32m+[m[32m      float dc = (dutyScale * tHigh2) / (float)tCycle;[m
[32m+[m[32m      float theta = (unitsFC - 1) - ((dc - dcMin) * unitsFC) / (dcMax - dcMin + 1);[m
     [m
[31m-    if(theta < 0.0)[m
[31m-        theta = 0.0;[m
[31m-    else if(theta > (unitsFC - 1.0))[m
[31m-        theta = unitsFC - 1.0;[m
[31m-  [m
[31m-    if((theta < q2min) && (thetaPre2 > q3max))[m
[31m-        turns2++;[m
[31m-    else if((thetaPre2 < q2min) && (theta > q3max))[m
[31m-        turns2--;[m
[31m-  [m
[31m-    if(turns1 >= 0)[m
[31m-        angle2 = (turns2 * unitsFC) + theta;[m
[31m-    else if(turns1 < 0)[m
[31m-        angle2 = ((turns2 + 1) * unitsFC) - (unitsFC - theta);[m
[31m-  [m
[31m-    thetaPre2 = theta;[m
[32m+[m[41m      [m
[32m+[m[32m      if(theta < 0.0)[m
[32m+[m[32m          theta = 0.0;[m
[32m+[m[32m      else if(theta > (unitsFC - 1.0))[m
[32m+[m[32m          theta = unitsFC - 1.0;[m
[32m+[m[41m    [m
[32m+[m[32m      if((theta < q2min) && (thetaPre2 > q3max))[m
[32m+[m[32m          turns2++;[m
[32m+[m[32m      else if((thetaPre2 < q2min) && (theta > q3max))[m
[32m+[m[32m          turns2--;[m
[32m+[m[41m    [m
[32m+[m[32m      if(turns1 >= 0)[m
[32m+[m[32m          angle2 = (turns2 * unitsFC) + theta;[m
[32m+[m[32m      else if(turns1 < 0)[m
[32m+[m[32m          angle2 = ((turns2 + 1) * unitsFC) - (unitsFC - theta);[m
[32m+[m[41m    [m
[32m+[m[32m      thetaPre2 = theta;[m
[32m+[m[32m    }[m
   }[m
 //-------------------------------------------------------------------[m
 [m
[31m-//Compute desired position and publish [m
[31m-  pos1 = pos1 + G_X*cmd1*(millis() - lasttime);[m
[31m-  pos2 = pos2 + G_Y*cmd2*(millis() - lasttime);[m
 [m
[31m-  servo1.write(constrain(((angle1-pos1)*KP) + 90, 0 , 180));[m
[31m-  servo2.write(constrain(((angle2-pos2)*KP) + 90, 0 , 180));[m
[32m+[m[32m  if(millis() > 200){[m
[32m+[m[32m    pos1 = pos1 + G_X*cmd1*(millis() - lasttime);[m
[32m+[m[32m    pos2 = pos2 + G_Y*cmd2*(millis() - lasttime);[m
   [m
[31m-  tpv_pos_x.data = angle1;[m
[31m-  pub1.publish(&tpv_pos_x);[m
[31m-[m
[31m-  tpv_pos_y.data = angle2;[m
[31m-  pub1.publish(&tpv_pos_y);[m
[32m+[m[32m    servo1.write(constrain(((angle1-pos1)*KP) + 90, 0 , 180));[m
[32m+[m[32m    servo2.write(constrain(((angle2-pos2)*KP) + 90, 0 , 180));[m
[32m+[m[41m    [m
[32m+[m[32m    tpv_pos_x.data = angle1;[m
[32m+[m[32m    pub1.publish(&tpv_pos_x);[m
   [m
[31m-  lasttime = millis();[m
[31m-  nh.spinOnce();[m
[31m-  delay(1);[m
[32m+[m[32m    tpv_pos_y.data = angle2;[m
[32m+[m[32m    pub1.publish(&tpv_pos_y);[m
[32m+[m[41m    [m
[32m+[m[32m    lasttime = millis();[m
[32m+[m[32m    nh.spinOnce();[m
[32m+[m[32m    delay(1);[m
[32m+[m[32m  }[m
[32m+[m[32m  else{[m
[32m+[m[32m    pos1 = angle1;[m
[32m+[m[32m    pos2 = angle2;[m
[32m+[m[32m  }[m
 }[m
 [m
[31m-// Interrupts for the 2 servo position sensor signal[m
[32m+[m
 void interupt1(){[m
   if(digitalRead(2)) {[m
     rise1 = micros();[m
