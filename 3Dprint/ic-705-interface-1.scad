/*
    IC-705 interface PCB box
    by OK1HRA, Creative Commons BY-SA
 -------------------------------------------------------------------
     How to export .stl
     After open press F5 for preview
     .STL export press F6 and F7 (or menu /Design/Export as STL...)

*/

 # translate([0,0,0]) rotate([0,0,0]) import("IC-705-interface-PCB-01.stl", convexity=3);
  
    difference(){
        union(){
            difference(){
                layout(0.2+2, 14.5+2);
                translate([0,0,-0.01]) layout(0.2-0.2, 14.5);
        }
        translate([0,0,1.6]) support(2, 13);
    }
    // -conn
    hull(){
        translate([40.5,64.2,3.22]) rotate([0,90,27]) cylinder(h=15, d=3.3+2, center=false, $fn=30);
        translate([43,58.85,3.22]) rotate([0,90,27]) cylinder(h=15, d=3.3+2, center=false, $fn=30);
    }
    translate([53,37.2,-0.1]) rotate([0,0,27]) cube([15,15.5,14.5]);
    translate([60,32.2,4.2]) rotate([0,90,27]) cylinder(h=15, d=4+2, center=false, $fn=30);
    translate([65,21.47,7.8]) rotate([0,90,27]) cylinder(h=15, d=7+2, center=false, $fn=30);
    }  
  
    module layout(SHIFT, ZZ){
        hull(){
            translate([2,72,0]) cylinder(h=ZZ, d=4+2*SHIFT, center=false, $fn=40);  
                translate([35.5,62,0]) cylinder(h=ZZ, d=24+2*SHIFT, center=false, $fn=140);  
            translate([2,2,0]) cylinder(h=ZZ, d=4+2*SHIFT, center=false, $fn=40);
            translate([35,0.5,0]) cylinder(h=ZZ, d=1+2*SHIFT, center=false, $fn=40);
            translate([68.35,19.28,0]) cylinder(h=ZZ, d=4+2*SHIFT, center=false, $fn=40);
        }
    }
    
    module support(SHIFT, ZZ){
        translate([2,74,0]) cylinder(h=ZZ, d=2*SHIFT, center=false, $fn=40);  
            translate([41.9,72,0]) cylinder(h=ZZ, d=2*SHIFT, center=false, $fn=40);  
        translate([0,30,0]) cylinder(h=ZZ, d=2*SHIFT, center=false, $fn=40);
        translate([25,0,0]) cylinder(h=ZZ, d=2*SHIFT, center=false, $fn=40);
        translate([47,6.13,0]) cylinder(h=ZZ, d=2*SHIFT, center=false, $fn=40);
        translate([55,10.2,0]) cylinder(h=ZZ, d=2*SHIFT, center=false, $fn=40);
        translate([60.55,39,0]) cylinder(h=ZZ, d=2*SHIFT, center=false, $fn=40);
    }